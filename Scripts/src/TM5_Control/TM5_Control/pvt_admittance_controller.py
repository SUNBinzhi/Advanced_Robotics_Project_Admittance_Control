import time
import mujoco as mj
import mujoco.viewer 
import numpy as np
import math 
import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import threading
import ikpy.chain
import serial
from tm_msgs.msg import FeedbackState



alpha = np.zeros([6,])
wrench_external = np.zeros(6)




"""
Mujoco Model Setup
"""
m = mj.MjModel.from_xml_path('/home/aaron/TM5/scene.xml')
m.opt.gravity[0] = 0.0  # x轴重力
m.opt.gravity[1] = 0.0  # y轴重力
m.opt.gravity[2] = 0.0  # z轴重力
d = mj.MjData(m)
m.opt.timestep =0.01

"""
Ikpy Setup
"""
# End Effector-joint6 to joint3
TM5_chain = ikpy.chain.Chain.from_urdf_file("/home/aaron/TM5/TM5_7.urdf")

"""
Admittance Algorithm
"""
# Mass Matrix
M_param = 1e1
M = np.diag([M_param,M_param,M_param,M_param,M_param,M_param])

# Damping Ratio
d_param =50 #50
d_param_rot =10 #50
D = np.diag([d_param, d_param, d_param, d_param_rot, d_param_rot, d_param_rot])

# K
k_param = 500 #100
k_param_rot = 20#100
K = np.diag([k_param, k_param, k_param, k_param_rot, k_param_rot, k_param_rot])

# Velocity and Acceleration Limit
arm_max_acc_ = 10.0  # Maximum acceleration
arm_max_vel_ = 1  # Maximum velocity




"""
Position Publisher
"""
class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'positions_topic', 10)
        time.sleep(1)  # 留出时间确保ROS 2网络准备就绪

    def publish_positions_with_duration(self, positions, duration,velocity):
        msg = Float64MultiArray()
        msg.data = positions + [duration] +velocity  # 将持续时间添加到位置数组的末尾
        self.publisher.publish(msg)
        # self.get_logger().info(f'Publishing: {msg.data}')
        
"""
Skin Data Subsriber
"""
class SensorDataSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_data_subscriber')
        self.data = None
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'force_data_topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        global force_data
        self.data = msg.data
        force_data = msg.data
        # self.get_logger().info(f'Publishing: {msg.data}')
        
"""
Joint Pose Subscriber
"""

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('demo_get_feedback')
        self.subscription = self.create_subscription(
            FeedbackState,
            'feedback_states',
            self.topic_callback,
            10)

    def topic_callback(self, msg):
        global joint_pos 
        if len(msg.joint_pos) == 6:
            joint_pos = msg.joint_pos
            
"""
Admittance Controller
"""      
# Admittance control function with fixed error shape assignment
def admittance_control_fixed(error, 
                             arm_position_, 
                             desired_pose_position_, 
                             desired_pose_orientation_, 
                             arm_orientation_, 
                             D, M, K, 
                             arm_desired_twist_adm_, 
                             dt, wrench_external_):
    # Position error
    error[:3] = np.around(arm_position_ - desired_pose_position_, decimals=3)

    # Orientation error
    # quat_rot_err = arm_orientation_ * desired_pose_orientation_.inv()
    # quat_rot_err_normalized = R.from_quat(quat_rot_err.as_quat() / np.linalg.norm(quat_rot_err.as_quat()))
    # print('error',quat_rot_err_normalized.as_rotvec())
    # print('error',quat_rot_err_normalized.as_rotvec().shape)
    # error[3:] = quat_rot_err_normalized.as_rotvec().reshape(3, 1)
    # # error[3:] = np.around( error[3:], decimals=3)
    # error[3:] = np.zeros([3,1])

    # Expected arm acceleration
    coupling_wrench_arm = D @ arm_desired_twist_adm_ + K @ error
    arm_desired_acceleration = np.linalg.inv(M) @ (wrench_external_ - coupling_wrench_arm)
    a_acc_norm = np.linalg.norm(arm_desired_acceleration[:3])
    # Acceleration limit check
    if a_acc_norm > arm_max_acc_:
        arm_desired_acceleration[:3] *= arm_max_acc_ / a_acc_norm

    # Update twist admittance
    arm_desired_acceleration = np.around( arm_desired_acceleration[:6], decimals=3)
    arm_desired_twist_adm_ += arm_desired_acceleration * dt

    # Velocity limit check
    a_vel_norm = np.linalg.norm(arm_desired_twist_adm_[:3])
    if a_vel_norm > arm_max_vel_:
        arm_desired_twist_adm_[:3] *= arm_max_vel_ / a_vel_norm
    arm_desired_twist_adm_=  np.around( arm_desired_twist_adm_[:6], decimals=2)

    linear_disp = (arm_desired_twist_adm_[:3]*dt ).flatten()
    angular_disp = (arm_desired_twist_adm_[3:]*dt).flatten()

    return arm_desired_twist_adm_,linear_disp,angular_disp,error

"""
IK Controller
"""

def forward_controller(chain,angles):
        # Ensure the matrix is a NumPy array
        T = chain.forward_kinematics([0,0,angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],0])
        T = np.array(T)
        # Extract the rotation matrix and translation vector
        rotation_matrix = T[:3, :3]
        translation_vector = T[:3, 3]
        # Convert the rotation matrix to Euler angles
        # euler_angles = R.from_matrix(rotation_matrix).as_matrix()
        # Translation vector gives the XYZ coordinates
        xyz_coordinates = translation_vector
        return rotation_matrix, xyz_coordinates

def inverse_controller(chain,pos,euler):
    if euler is None:
        euler = np.array([[ 3.26794897e-07,  3.26794789e-07, -1.00000000e+00],
                        [-1.00000000e+00 , 3.26794895e-07 ,-3.26794790e-07],
                        [ 3.26794789e-07  ,1.00000000e+00 , 3.26794896e-07]])
    result = TM5_chain.inverse_kinematics(pos,euler,"all")[2:8]
    return result


"""
Skin Data Recieve Through WIFI
"""

def calculate_checksum(data):
    """计算校验和，返回低八位"""
    return sum(data) & 0xFF

def parse_data_packet(packet):
    """解析数据包"""
    if len(packet) != 35 or packet[0] != 0xAA or packet[1] != 0x01:
        print("无效的数据包")
        return None

    values = [packet[i] * 256 + packet[i + 1] for i in range(2, 34, 2)]
    checksum = calculate_checksum(packet[:-1])
    if checksum != packet[-1]:
        print("校验和不匹配")
        return None

    return values




rclpy.init(args=None)
positions_publisher = PositionPublisher()
minimal_subscriber = MinimalSubscriber()
skin_subscriber = SensorDataSubscriber()



"""
Skin Data Calculate Thread
"""

def Skin_Thread():
    global wrench_external,force_data
    force_direction = np.zeros((11, 11), dtype=object)
    force = np.zeros((11, 11), dtype=object)

    # try:
    # force_data_ = force_data-force_data

    while 1:
        try:
            rclpy.spin_once(skin_subscriber)
            for l in range(len(force_data)):
                        if force_data[l] <=  500:
                            force_data[l] =0
                        else:
                            pass
            data = np.array(force_data).reshape([11,11])/30
            # data = np.clip(data-1000,0,1000)/50
            # print(data)
            # print('raw_date_handled',data)
            
            for i in range(11):
                for j in range(11):
                    force_direction[i,j] =  np.array([0,-math.cos(2*math.pi/2-j*math.pi/11),-math.sin(2*math.pi/2-j*math.pi/11)])
                    force_direction[i,j] =  np.array([0,-math.cos(1*math.pi*7/6-j*math.pi*7/66-math.pi/6),-math.sin(1*math.pi*7/6-j*math.pi*7/66-math.pi/6)])
            for i in range(11):
                for j in range(11):
                    force[i,j] = np.array(force_direction[i,j] ) * data[i,j]
            # wrench_external = force
            force_y = 0
            force_z = 0
            for i in range(11):
                for j in range(11):
                    force_y += force[i, j][1]  # 累加每个元素的第二个数
                    force_z += force[i, j][2]  # 累加每个元素的第三个数
            wrench_external[1] = force_y
            wrench_external[0] = force_z
            # print(force)
        except:
            pass


"""
Joint Pose  Feed Back Thread
"""
def get_feedback():
    global joint_pos
    rclpy.spin(minimal_subscriber)
        
"""
Admittance Controller Loop Thread
"""
# positions_publisher.publish_positions_with_duration(positions, duration)

def Admittance_Controller(chain,D_pos,D_ori):
    global wrench_external
    global joint_pos
    
    # initial_joint_pos = np.array([-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00, 1.56651260e+00, -1.95721289e-12])
    initial_pos = np.array([-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00+math.pi/2, 1.56651260e+00, -1.95721289e-12])
    error = np.zeros((6,1))
    command_pos = np.zeros((6,1))
    arm_desired_twist_adm_ = np.zeros((6,1))
    desired_position = D_pos
    desired_orientation = D_ori
    links_len = len(chain.links)
    # Algorithem Frequency
    dt = 0.03
    
    for i in range(100):
        positions_publisher.publish_positions_with_duration(desired_position,1,velocity=[0.0,0.0,0.0,0.0,0.0,0.0])
        time.sleep(0.1)
    
    with mj.viewer.launch_passive(m, d) as viewer:
        if wrench_external is not None:
            while 1:
                current_ee_position,current_ee_orientation = forward_controller(chain,joint_pos[0:links_len-3])
                current_ee_position = current_ee_position.reshape(-1,1)
                current_ee_orientation = R.from_euler('xyz',current_ee_orientation)
                arm_desired_twist_adm_,linear_disp,angular_disp,error= admittance_control_fixed(error, 
                            current_ee_position, 
                            desired_position, 
                            desired_orientation, 
                            current_ee_orientation, 
                            D, M, K, 
                            arm_desired_twist_adm_, dt,
                            wrench_external.reshape([-1,1]))
                
                current_angular,current_linear= forward_controller(chain,joint_pos[0:links_len-3])
                new_linear = current_linear +linear_disp.reshape([3,])
                fix_angular,fix_linear = forward_controller(chain,initial_joint_pos[0:links_len-3])
                new_angular = fix_angular
                d.ctrl[0:links_len-3] = joint_pos
                command_pos[:3] = new_linear
                command_pos[4:6]  = angular
                positions_publisher.publish_positions_with_duration(command_pos,dt,velocity=[0.0,0.0,0.0,0.0,0.0,0.0])
                viewer.sync()
                mj.mj_step(m, d)

def Mujoco_Admittance_Controller(chain,D_pos,D_ori):

    global wrench_external
    global joint_pos
    global force_data

    # Parameter Init
    t = 0
    error = np.zeros((6, 1))
    arm_desired_twist_adm_ = np.zeros((6, 1))
    desired_pose_position_ = D_pos
    desired_pose_orientation_ = R.from_quat(D_ori)
    links_len = len(chain.links)
    initial_pos = np.array([-0.00167857, -0.17282314 , 1.38263255 , 0.36098724 , 1.57079633 ,-0.00167792])

    #start simulation
    for i in range(1000):
        # viewer.sync()   
        d.ctrl = initial_pos
        mj.mj_step(m, d)
        time.sleep(0.001)

    with mj.viewer.launch_passive(m, d) as viewer:
        while 1:  
            viewer.sync()  
            start_time = time.time()
            t = t+1
            error = np.zeros((6, 1))
            d.xfrc_applied[links_len-2] = wrench_external 
            # wrench_external = d.xfrc_applied[links_len-2] 
            # print(wrench_external)
            # wrench_external[2] = 50
            
     
            arm_orientation_,arm_position_ = forward_controller(chain,d.qpos)
            arm_position_ = arm_position_.reshape(-1, 1)
            arm_orientation_ = R.from_matrix(arm_orientation_)
            arm_desired_twist_adm_,linear_disp,angular_disp,error = admittance_control_fixed(error, 
                            arm_position_, 
                            desired_pose_position_, 
                            desired_pose_orientation_, 
                            arm_orientation_, 
                            D, M, K, 
                            arm_desired_twist_adm_, m.opt.timestep,
                            wrench_external.reshape([-1,1]))
            current_angular,current_linear= forward_controller(chain,d.qpos)
            new_linear = current_linear +linear_disp.reshape([3,])
            fix_angular,fix_linear = forward_controller(chain,initial_pos[0:links_len-3])
            new_angular = fix_angular
            d.ctrl[0:links_len-3] = inverse_controller(chain,new_linear,new_angular)
 
            mj.mj_step(m, d)
            elapsed_time = time.time() - start_time
            remaining_time = max(0.01 - elapsed_time, 0)
            time.sleep(remaining_time)
            # time.sleep(0.01)               
             
def main():
    chain = TM5_chain
    D_Pos = np.array([ 0.41700019, -0.12300043,  0.60000069]).reshape([3,1])
    D_Ori = [7.07106780e-01, 7.07106782e-01, 2.40220236e-09, 1.39964104e-10]

    # thread1 = threading.Thread(target=get_feedback,args=())
    thread2 = threading.Thread(target=Skin_Thread,args=())
    thread3 = threading.Thread(target=Mujoco_Admittance_Controller, args=(chain,D_Pos,D_Ori))
    # thread4 = threading.Thread(target=TM5_Thread, args=())

    # Thread Start 
    # thread1.start()
    thread2.start()
    thread3.start()
    # thread4.start()

    # Wait for finishing
    # thread1.join()
    thread2.join()
    thread3.join()
    # thread4.join()
    
if __name__ == '__main__':
    main()