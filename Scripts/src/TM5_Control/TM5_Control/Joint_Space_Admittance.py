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
TM5_Joint_7 = ikpy.chain.Chain.from_urdf_file("/home/aaron/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/script/Different_Joint_Test/TM5_7.urdf")
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
d_param =40 #50
d_param_rot =10 #50
D = np.diag([d_param, d_param, d_param, d_param_rot, d_param_rot, d_param_rot])
D = d_param

# K
k_param = 80 #100
k_param_rot = 20#100
K = np.diag([k_param, k_param, k_param, k_param_rot, k_param_rot, k_param_rot])
K = k_param

# Velocity and Acceleration Limit
arm_max_acc_ = 100.0  # Maximum acceleration
arm_max_vel_ = 10.0  # Maximum velocity

alpha = np.array([-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00+math.pi/2, 1.56651260e+00, -1.95721289e-12])
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
def admittance_control_fixed(d,error, 
                             arm_position_, 
                             desired_pose_position_, 
                             desired_pose_orientation_, 
                             arm_orientation_, 
                             D, M, K, 
                             arm_desired_twist_adm_, 
                             dt, wrench_external_):
    # Position error
    # print('arm_position_:=',arm_position_)
    # print('desired_pose_position_:=',desired_pose_position_)
    arm_position_ = np.array(arm_position_)
    desired_pose_position_ = np.array(desired_pose_position_)
    error = np.around(arm_position_.reshape([6,1])- desired_pose_position_.reshape([6,1]), decimals=3)

    # Orientation error
    # quat_rot_err = arm_orientation_ * desired_pose_orientation_.inv()
    # quat_rot_err_normalized = R.from_quat(quat_rot_err.as_quat() / np.linalg.norm(quat_rot_err.as_quat()))
    # error[3:] = quat_rot_err_normalized.as_rotvec().reshape(3, 1)
    # error[3:] = np.around( error[3:], decimals=3)
    # print('error:=',error.reshape(6,))


    # Expected arm acceleration
    # print(f'arm_desired_twist_adm_: {arm_desired_twist_adm_.shape}')
    # print(f'error: {error.shape}')
    # print('----------------------------------------------')
    coupling_wrench_arm = D * arm_desired_twist_adm_ + K * error
    # print(f'wrench_external_: {wrench_external_.shape}')

    arm_desired_acceleration = np.linalg.inv(M) @ (wrench_external_ - coupling_wrench_arm)
    a_acc_norm = np.linalg.norm(arm_desired_acceleration[:3])
    # Acceleration limit check
    if a_acc_norm > arm_max_acc_:
        arm_desired_acceleration[:3] *= arm_max_acc_ / a_acc_norm
    # print('acc',arm_desired_acceleration[:3].reshape([3,]))

    # Update twist admittance
    # print(arm_desired_acceleration.shape)
    arm_desired_acceleration = np.around( arm_desired_acceleration[:6], decimals=3)
    arm_desired_twist_adm_ += arm_desired_acceleration * dt
    # print('arm_desired_acceleration',arm_desired_acceleration.reshape(6,))
    # print('----------------------------------------------')
    # Velocity limit check
    a_vel_norm = np.linalg.norm(arm_desired_twist_adm_[:3])
    if a_vel_norm > arm_max_vel_:
        arm_desired_twist_adm_[:3] *= arm_max_vel_ / a_vel_norm
    arm_desired_twist_adm_=  np.around( arm_desired_twist_adm_[:6], decimals=2)
    # Delta linear displacement and angular displacement
    linear_disp = (arm_desired_twist_adm_[:3]*dt ).flatten()
    angular_disp = (arm_desired_twist_adm_[3:]*dt).flatten()


    return arm_desired_twist_adm_,linear_disp,angular_disp,error


"""
IK Controller
"""

def forward_controller(chain,angles):
        # Ensure the matrix is a NumPy array
        links_len = len(chain.links) 
        forward_input = np.zeros([links_len,])
        forward_input[2:links_len-1] = angles
        T = chain.forward_kinematics(forward_input)
        T = np.array(T)
        # Extract the rotation matrix and translation vector
        rotation_matrix = T[:3, :3]
        translation_vector = T[:3, 3]
        # Convert the rotation matrix to Euler angles
        euler_angles = R.from_matrix(rotation_matrix).as_matrix()
        # Translation vector gives the XYZ coordinates
        xyz_coordinates = translation_vector
        return euler_angles, xyz_coordinates


def inverse_controller(chain,pos,euler):
    links_len = len(chain.links) 
    result = chain.inverse_kinematics(pos,euler,"all")[2:links_len-1]
    return result

"""
Jacobian_Calculate
"""
def calculate_jacobian(T_matrices):
    """
    计算雅克比矩阵。
    
    :param T_matrices: 包含T01, T02, ..., T06的列表
    :return: 雅克比矩阵
    """
    n = len(T_matrices)
    J = np.zeros((6, n))
    
    # 末端执行器的位置
    end_effector_pos = T_matrices[-1][:3, 3]
    
    for i in range(n):
        # 当前关节的旋转矩阵和位置
        R_i = T_matrices[i][:3, :3]
        d_i = T_matrices[i][:3, 3]
        
        # 计算z轴和p轴
        z = R_i[:, 2]
        p = end_effector_pos - d_i
        
        # 计算雅克比矩阵的列
        J[:3, i] = np.cross(z, p)
        J[3:, i] = z
    
    return J

"""
Local Wrench to Global Wrench
"""

def transform_force_and_torque(Wrench, transformation_matrix):

    local_force = np.array([Wrench[1], Wrench[2], Wrench[0]])  # y, z, x
    local_torque = np.array([Wrench[4], Wrench[5], Wrench[3]])  # ry, rz, rx
    # local_force = Wrench[:3]  # 取前三个元素作为力
    # local_torque = Wrench[3:]  # 取后三个元素作为力矩
    # 提取变换矩阵中的旋转矩阵部分
    rotation_matrix = transformation_matrix[0:3, 0:3]

    # 转换力和力矩
    global_force = rotation_matrix @ local_force
    global_torque = rotation_matrix @ local_torque


    global_force_torque = np.concatenate((global_force, global_torque))

    return global_force_torque

"""
Main Loop Thread
"""

def Mujoco_Thread(chain,D_Pos,D_Ori):
    global alpha
    global wrench_external
    global joint_pos
    initial_pos = np.array([-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00+math.pi/2, 1.56651260e+00, -1.95721289e-12])

    # Parameter Init
    t = 0
    error = np.zeros((6, 1))
    arm_desired_twist_adm_ = np.zeros((6, 1))
    desired_pose_position_ = initial_pos
    desired_pose_orientation_ = D_Ori
    links_len = len(chain.links)

    #start simulation
    for i in range(1000):
        # viewer.sync()   
        d.ctrl = initial_pos
        alpha = initial_pos
        mj.mj_step(m, d)
        # time.sleep(0.002)

    with mj.viewer.launch_passive(m, d) as viewer:
        while 1:  
            viewer.sync()  
            t = t+1
            error = np.zeros((6, 1))

            #Jacobian Calculate
            joints = np.zeros(9)
            try:
                joints[2:links_len-1] = joint_pos #d.qpos
            except:
                joint_pos = initial_pos
                joints[2:links_len-1]  = initial_pos
            All_matrix = chain.forward_kinematics(joints,True)[2:links_len-1]
            J = calculate_jacobian(All_matrix)

            # Wrench External Transform
            Force = wrench_external
            Force= transform_force_and_torque(Force,All_matrix[-1])
            # d.xfrc_applied[7] = wrench_external.reshape([6,])
            wrench_external_ = (J.T @ Force).reshape(-1, 1)
            wrench_external_ = np.clip(wrench_external_, -300, 300)
            print('Force Applied:',wrench_external)

            arm_orientation_,arm_position_ = forward_controller(chain,joint_pos[0:links_len-3])
            # arm_position_ = arm_position_.reshape(-1, 1)joint_pos
            # arm_position_ = joint_pos
            arm_position_ = joint_pos #d.qpos
            arm_orientation_ = R.from_euler('xyz',arm_orientation_)
            arm_desired_twist_adm_,linear_disp,angular_disp,error = admittance_control_fixed(d,error, 
                            arm_position_, 
                            desired_pose_position_, 
                            desired_pose_orientation_, 
                            arm_orientation_, 
                            D, M, K, 
                            arm_desired_twist_adm_, m.opt.timestep,
                            wrench_external_)
            
            # Current_Joints = joint_pos
            Current_Joints = joint_pos #d.qpos
            Delta_Joints = (arm_desired_twist_adm_*m.opt.timestep).reshape([6,])
            New_Joints = Current_Joints+Delta_Joints
            New_Joints[3] = -(New_Joints[2]+New_Joints[1]-math.pi/2)
            # New_Joints[4] = -(New_Joints[0]-math.pi/2)
            # Safety Control
            New_Joints[0] = np.clip(New_Joints[0], -0.5, 0.5)
            New_Joints[1] = np.clip(New_Joints[1], -0.6, -0.4)
            New_Joints[2] = np.clip(New_Joints[2], 1.3, 2)
            # New_Joints[3] = np.clip(New_Joints[3], -1.7, -1)
            d.ctrl[0:links_len-3]  = New_Joints
            alpha = d.ctrl


            mj.mj_step(m, d)
            # time.sleep(0.01)

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
            # print('6666666666666',force_data)
            # force_data_fix = np.zeros((11, 11), dtype=object)
            # force_data_fix =np.clip(-force_data+2000,-10000,0)
            # print('777777777777777',force_data_fix)

            # data = np.array(skin_subscriber.data).reshape([11,1S1])/100
            data = np.array(force_data).reshape([11,11])
            data = np.clip(data-1000,0,1000)/50
            # print('Force Applied:',force_data)

            # print(data)
            # data = data
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

rclpy.init(args=None)
positions_publisher = PositionsPublisher()
minimal_subscriber = MinimalSubscriber()
skin_subscriber = SensorDataSubscriber()
# joint_pos = np.zeros(6)
"""
Joint Pose Thread
"""
def get_feedback():
    global joint_pos
    rclpy.spin(minimal_subscriber)
    # print(joint_pos)

def main():
    # control_loop()
    # Thread Create
    mode =1
    if mode == 1:
        chain = TM5_Joint_7
        D_Pos = np.array([ 0.27629562, -0.12396941,  0.82000005]).reshape([3,1])
        D_Ori = [-5.02137269e-01, -5.02137270e-01 ,-4.97853556e-01, -4.97853555e-01]
    elif mode == 2: 
        chain = TM5_Joint_6
        D_Pos = np.array([ 0.16314977 ,-0.12300002 , 0.82000005]).reshape([3,1])
        D_Ori = [-7.10129337e-01, -3.55775981e-10 ,-3.55775859e-10, -7.04071250e-01]       
    elif mode == 3:
        chain = TM5_Joint_5
        D_Pos = np.array([ 0.16314977, -0.12300002 , 0.71400005]).reshape([3,1])
        D_Ori = [-7.07105159e-01 , 7.07105159e-01 ,-1.51452553e-03 , 1.51452482e-03]  
    elif mode == 4:
        chain = TM5_Joint_4
        D_Pos = np.array([ -0.20134143 , 0.0008625 ,  0.52401643]).reshape([3,1])
        D_Ori = [-6.86487028e-01,  6.87206886e-01 , 1.66573393e-01,  1.69515663e-01]  
    thread1 = threading.Thread(target=Skin_Thread,args=())
    thread2 = threading.Thread(target=get_feedback,args=())
    thread4 = threading.Thread(target=TM5_Thread, args=())
    thread3 = threading.Thread(target=Mujoco_Thread, args=(chain,D_Pos,D_Ori))

    # Thread Start 
    thread1.start()
    thread2.start()
    thread4.start()
    thread3.start()

    # Wait for finishing
    thread1.join()
    thread2.join()
    thread4.join()
    thread3.join()

if __name__ == '__main__':
    main()

