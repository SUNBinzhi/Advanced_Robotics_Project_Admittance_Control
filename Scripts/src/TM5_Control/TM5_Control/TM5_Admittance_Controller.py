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


"""
Mujoco Model Setup
"""
m = mj.MjModel.from_xml_path('/home/aaron/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/TM5-900_description/TM5/scene.xml')
m.opt.gravity[0] = 0.0  # x轴重力
m.opt.gravity[1] = 0.0  # y轴重力
m.opt.gravity[2] = 0.0  # z轴重力
d = mj.MjData(m)
m.opt.timestep = 0.01


"""
Ikpy Setup
"""
# End Effector-joint6 to joint3
TM5_chain = ikpy.chain.Chain.from_urdf_file("/home/aaron/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/script/Different_Joint_Test/TM5_7.urdf")
TM5_Joint_6 = ikpy.chain.Chain.from_urdf_file("/home/aaron/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/script/Different_Joint_Test/TM5_6.urdf")
TM5_Joint_5 = ikpy.chain.Chain.from_urdf_file("/home/aaron/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/script/Different_Joint_Test/TM5_5.urdf")
TM5_Joint_4 = ikpy.chain.Chain.from_urdf_file("/home/aaron/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/script/Different_Joint_Test/TM5_4.urdf")

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
arm_max_acc_ = 100.0  # Maximum acceleration
arm_max_vel_ = 10.0  # Maximum velocity

alpha = np.zeros([6,])
wrench_external = np.zeros(6)

"""
Control Publisher
"""
class PositionsPublisher(Node):
    def __init__(self):
        super().__init__('positions_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'positions_topic', 10)

    def publish_positions(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
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
        self.get_logger().info(f'Publishing: {msg.data}')

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
def admittance_control_fixed(d,error, 
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
        euler_angles = R.from_matrix(rotation_matrix).as_matrix()
        # Translation vector gives the XYZ coordinates
        xyz_coordinates = translation_vector
        return euler_angles, xyz_coordinates

def inverse_controller(pos,euler):
    if euler is None:
        euler = np.array([[ 3.26794897e-07,  3.26794789e-07, -1.00000000e+00],
                        [-1.00000000e+00 , 3.26794895e-07 ,-3.26794790e-07],
                        [ 3.26794789e-07  ,1.00000000e+00 , 3.26794896e-07]])
    result = TM5_chain.inverse_kinematics(pos,euler,"all")[2:8]
    return result


"""
Main Loop Thread
"""

def Mujoco_Thread():
    global alpha
    global wrench_external
    global joint_pos
    error = np.zeros((6, 1))
    arm_desired_twist_adm_ = np.zeros((6, 1))

    initial_pos = np.array([-4.28372736e-03 ,-4.88533739e-01,  1.57943555e+00, -1.09090181e+00,
                                    -1.56651260e+00 ,-1.95721289e-12])
    #start simulation
    m.opt.timestep = 0.01
    # while viewer.is_running():
    for i in range(1000):
        # viewer.sync()   
        d.ctrl = initial_pos
        alpha = initial_pos
        initial_orienctation,initial_xyz = forward_controller(TM5_chain,initial_pos)
        initial_orienctation = R.from_euler('xyz',initial_orienctation).as_quat()
        desired_pose_position_ = initial_xyz.reshape([3,1]) # Replace with your target position
        desired_pose_orientation_ = R.from_quat(initial_orienctation ) # Replace with your target orientation
        mj.mj_step(m, d)

        # time.sleep(0.002)

    with mj.viewer.launch_passive(m, d) as viewer:

        while 1:  
            viewer.sync()  
            error = np.zeros((6, 1))
            d.xfrc_applied[7] = wrench_external 
            # var_force = round(random.uniform(0, 100), 1)
            # wrench_external = np.array([0, 0, var_force, 0, 0, 0])
            # env.data.xfrc_applied[7] =  wrench_external #read external forces in mujoco 
            # wrench_external = env.data.xfrc_applied[7]
            # env.data.xfrc_applied[7] =  wrench_external #read external forces in mujoco 

            #############################################################
            #              Parameter set for Admittance Control         #
            #############################################################

            arm_position_ = np.array(d.xpos[7]).reshape(-1, 1)  # Current arm end effector position
            # print(f"initial_position{arm_position_}")
            # print('qpos:',d.qpos)
            # arm_orientation_,nothing = forward_controller(TM5_chain,d.qpos)
            arm_orientation_,nothing = forward_controller(TM5_chain,joint_pos)
            arm_orientation_ = R.from_euler('xyz',arm_orientation_)
            # print(error)
            arm_desired_twist_adm_,linear_disp,angular_disp,error = admittance_control_fixed(d,error, 
                            arm_position_, 
                            desired_pose_position_, 
                            desired_pose_orientation_, 
                            arm_orientation_, 
                            D, M, K, 
                            arm_desired_twist_adm_, m.opt.timestep,
                            d.xfrc_applied[7].reshape(-1, 1))
            current_angular,current_linear= forward_controller(TM5_chain,joint_pos)
            # current_angular,current_linear= forward_controller(TM5_chain,d.qpos)
            new_linear = current_linear +linear_disp.reshape([3,])
            # new_angular = current_angular-angular_disp.reshape([3,])
            new_angular =  np.array([[ 3.26794897e-07,  3.26794789e-07, -1.00000000e+00],
                                        [-1.00000000e+00 , 3.26794895e-07 ,-3.26794790e-07],
                                        [ 3.26794789e-07  ,1.00000000e+00 , 3.26794896e-07]])

            d.ctrl = inverse_controller(new_linear,new_angular)
            alpha = d.ctrl
            mj.mj_step(m, d)

            time.sleep(0.01)

"""
Command Send Thread
"""
def TM5_Thread():
    global alpha
    while 1:
        positions_publisher.publish_positions(list(alpha))
        time.sleep(0.3)

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


"""
Soft Sensor data Calcualte Thread
"""
def serial_read_thread(port, baudrate):
    global wrench_external
    ser = serial.Serial(port, baudrate, timeout=1)
    try:
        while True:
            if ser.in_waiting:
                packet = ser.read(35)
                packet = list(packet)
                data_values = parse_data_packet(packet)
                if data_values is not None:
                    data_values[0] = data_values[0]-63
                    data_values[1] = data_values[1]-32
                    data_values[2] = data_values[2]-0
                    data_values[3] = data_values[3]-0
                    data_values[4] = data_values[4]-245
                    data_values[5] = data_values[5]-285
                    data_values[6] = data_values[6]-260
                    data_values[7] = data_values[7]-75
                    data_values[8] = data_values[8]-0
                    data_values[9] = data_values[9]-0
                    data_values[10] = data_values[10]-0
                    data_values[11] = data_values[11]-34
                    data_values[12] = data_values[12]-0
                    data_values[13] = data_values[13]-0
                    data_values[14] = data_values[14]-0
                    data_values[15] = data_values[15]-30
                    # wrench_external = np.array([data_values[4]/20, data_values[6]/20, data_values[1]/20, 0, 0, 0])
                    data_values_1 = np.array([0.0,0.9239*(data_values[0]+data_values[4]+data_values[8]+data_values[12]),-0.3827*(data_values[0]+data_values[4]+data_values[8]+data_values[12])])
                    data_values_2 = np.array([0.0,0.3827*(data_values[1]+data_values[5]+data_values[9]+data_values[13]),-0.9239*(data_values[1]+data_values[5]+data_values[9]+data_values[13])])
                    data_values_3 = np.array([0.0,-0.3827*(data_values[2]+data_values[6]+data_values[10]+data_values[14]),-0.9239*(data_values[2]+data_values[6]+data_values[10]+data_values[14])])
                    data_values_4 = np.array([0.0,-0.9239*(data_values[3]+data_values[7]+data_values[11]+data_values[15]),-0.3927*(data_values[3]+data_values[7]+data_values[11]+data_values[1])])
                    
                    wrench_external = np.array([0, (data_values_1[1]+data_values_2[1]+data_values_3[1]+data_values_4[1])/10, (data_values_1[2]+data_values_2[2]+data_values_3[2]+data_values_4[2])/10, 0, 0, 0])
                    # print(wrench_external)
    finally:
        ser.close()


"""
Skin Data Calculate Thread
"""

def Skin_Thread():
    global wrench_external,force_data
    force_direction = np.zeros((11, 11), dtype=object)
    force = np.zeros((11, 11), dtype=object)
    # try:
    # print(skin_subscriber.data)
    while 1:
        try:
            rclpy.spin_once(skin_subscriber)
            # data = np.array(skin_subscriber.data).reshape([11,1S1])/100
            data = np.array(force_data).reshape([11,11])/80
            for i in range(11):
                for j in range(11):
                    force_direction[i,j] =  np.array([0,-math.cos(2*math.pi/3-j*math.pi/33),-math.sin(2*math.pi/3-j*math.pi/33)])
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
            wrench_external[1] = -force_y
            wrench_external[2] = force_z
        except:
            pass

rclpy.init(args=None)
positions_publisher = PositionsPublisher()
minimal_subscriber = MinimalSubscriber()
skin_subscriber = SensorDataSubscriber()

"""
Joint Pose Thread
"""
def get_feedback():
    global joint_pos
    rclpy.spin(minimal_subscriber)
    print(joint_pos)

def main():
    # control_loop()
    # Thread Create
    thread1 = threading.Thread(target=Mujoco_Thread, args=())
    thread2 = threading.Thread(target=TM5_Thread, args=())
    thread3 = threading.Thread(target=Skin_Thread,args=())
    thread4 = threading.Thread(target=get_feedback,args=())

    # Thread Start 
    thread4.start()
    thread3.start()
    thread1.start()
    thread2.start()

    # Wait for finishing
    thread4.join()
    thread3.join()
    thread1.join()
    thread2.join()

if __name__ == '__main__':
    main()

