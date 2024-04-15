import time
import mujoco as mj
import mujoco.viewer 
import numpy as np
import math 
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
import threading
import ikpy.chain
# import serial
#Sensor imports
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import numpy as np
import time
import threading
from math import pi, sin,cos


TM5_chain = ikpy.chain.Chain.from_urdf_file(r"C:\Users\15738\Desktop\AR_project\Advanced_Robotics_Project-1\Scripts\TM5.urdf")

# Basic Parameters for the End-Effector
M_param = 1e1
M = np.diag([M_param,M_param,M_param,M_param,M_param,M_param])

d_param =100 #50
d_param_rot =100 #50
D = np.diag([d_param, d_param, d_param, d_param_rot, d_param_rot, d_param_rot])
k_param =200 #100
k_param_rot = 200#100
K = np.diag([k_param, k_param, k_param, k_param_rot, k_param_rot, k_param_rot])
arm_max_acc_ = 10.0  # Maximum acceleration
arm_max_vel_ = 0.5  # Maximum velocity

m = mj.MjModel.from_xml_path(r'C:\Users\15738\Desktop\AR_project\Advanced_Robotics_Project-1\TM5_Description\scene_velocity.xml')
m.opt.gravity[0] = 0.0  # x轴重力
m.opt.gravity[1] = 0.0  # y轴重力
m.opt.gravity[2] = 0.0  # z轴重力
d = mj.MjData(m)







def admittance_control_fixed(d,error, 
                             arm_position_, 
                             desired_pose_position_, 
                             desired_pose_orientation_, 
                             arm_orientation_, 
                             D, M, K, 
                             arm_desired_twist_adm_, 
                             dt, wrench_external_):
    # Position error
    error[:3] = arm_position_ - desired_pose_position_

    # Orientation error
    quat_rot_err = arm_orientation_ * desired_pose_orientation_.inv()
    quat_rot_err_normalized = R.from_quat(quat_rot_err.as_quat() / np.linalg.norm(quat_rot_err.as_quat()))
    error[3:] = quat_rot_err_normalized.as_rotvec().reshape(3, 1)
    error[3:] = error[3:]
    # print('error:=',error.reshape(6,))
    # print('----------------------------------------------')


    # Expected arm acceleration
    coupling_wrench_arm = D @ arm_desired_twist_adm_ + K @ error
    arm_desired_acceleration = np.linalg.inv(M) @ (wrench_external_ - coupling_wrench_arm)
    a_acc_norm = np.linalg.norm(arm_desired_acceleration[:3])
    # Acceleration limit check
    if a_acc_norm > arm_max_acc_:
        arm_desired_acceleration[:3] *= arm_max_acc_ / a_acc_norm
    # print('acc',arm_desired_acceleration[:3].reshape([3,]))

    # Update twist admittance
    # print(arm_desired_acceleration.shape)
    arm_desired_acceleration = arm_desired_acceleration[:6]
    arm_desired_twist_adm_ += arm_desired_acceleration * dt
    # print('arm_desired_acceleration',arm_desired_acceleration.reshape(6,))
    # print('----------------------------------------------')


    # Velocity limit check
    a_vel_norm = np.linalg.norm(arm_desired_twist_adm_[:3])
    if a_vel_norm > arm_max_vel_:
        arm_desired_twist_adm_[:3] *= arm_max_vel_ / a_vel_norm
    arm_desired_twist_adm_= arm_desired_twist_adm_[:6]
    # Delta linear displacement and angular displacement
    linear_disp = (arm_desired_twist_adm_[:3]*dt ).flatten()
    angular_disp = (arm_desired_twist_adm_[3:]*dt).flatten()


    return arm_desired_twist_adm_,error

def forward_controller(chain,angles):
        # Ensure the matrix is a NumPy array
        T = chain.forward_kinematics([0,0,angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],0])
        T = np.array(T)
        # Extract the rotation matrix and translation vector
        rotation_matrix = T[:3, :3]
        translation_vector = T[:3, 3]
        # Convert the rotation matrix to Euler angles
        euler_angles = rotation_matrix
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

def calculate_pseudo_inverse(J, lambda_reg=0.01):
    """
    计算带有正则化的伪逆。
    """
    JTJ = J.T @ J
    reg_identity = np.eye(JTJ.shape[0]) * lambda_reg
    inv = np.linalg.inv(JTJ + reg_identity)
    return inv @ J.T

def limit_joint_velocities(joint_velocities, max_velocity):
    """
    限制关节速度。
    """
    norm_velocities = np.linalg.norm(joint_velocities)
    if norm_velocities > max_velocity:
        return joint_velocities * (max_velocity / norm_velocities)
    return joint_velocities



def reinterpredSigned16(val_array):
    for idx, val in enumerate(val_array):
        if val >= 32768:
            val_array[idx] = val - 65536


class ModbusRobotIQ:
    def __init__(self, method="rtu", port="COM3", stopbits=1, bytesize=8, parity='N', baudrate=19200):
        self.client = ModbusClient(method=method, port=port, stopbits=stopbits,
                                   bytesize=bytesize, parity=parity, baudrate=baudrate)

        # print(connection)
        self.lock = threading.Lock()

    def connect(self):
        connection = self.client.connect()
        print(f'[ft sensor] connection: {connection}')

    def disconnect(self):
        self.client.close()
        print(f'[ft sensor] disconnect')

    def get_data(self):

        results = np.zeros([1, 6]).reshape([1, 6])
        # print(results.shape)

        try:
            self.lock.acquire()
            # print(self.client)
            request = self.client.read_holding_registers(address=180, count=6, slave=0, unit=9)
            # print(request)
            raw_results = np.array(request.registers)
            reinterpredSigned16(raw_results)
            results = np.array(raw_results).reshape([1, 6])
            # print(results.shape)# .flatten().tolist()
            # print(type(results))
            self.lock.release()
        except:
            print(f'[modbus_robotiq]: got empty position array')
            pass

        return results

def Mujoco_Thread():
    links_len = len(TM5_chain.links)

    error = np.zeros((6, 1))
    arm_desired_twist_adm_ = np.zeros((6, 1))

    initial_pos = [-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00+pi/2, 1.56651260e+00, -1.95721289e-12]
    #  np.array([-4.28372736e-03 ,-4.88533739e-01,  1.57943555e+00, -1.09090181e+00,
    #                                 -1.56651260e+00 ,-1.95721289e-12])
    # initial_pos = np.array([0,0,0,0,0,0])
    #start simulation
    m.opt.timestep = 0.05

    initial_orientation, initial_xyz = forward_controller(TM5_chain, initial_pos)
    # print('initial ori',initial_orientation)
                                                #     initial ori [[ 1.00000000e+00 -2.13589791e-13  3.26794897e-07]
                                                #  [ 3.26794897e-07  6.53589686e-07 -1.00000000e+00]
                                                #  [ 1.69886486e-20  1.00000000e+00  6.53589686e-07]]
    initial_orientation = R.from_matrix(initial_orientation)
    desired_pose_position_ = initial_xyz.reshape([3, 1])
    desired_pose_orientation_ = initial_orientation
    count = 0
    
    """Sensor"""
    client_ft = ModbusRobotIQ(method="rtu", port="COM3", stopbits=1, bytesize=8, parity='N', baudrate=19200)
    time.sleep(2)
    initial_force_data = client_ft.get_data()
    
    with mj.viewer.launch_passive(m, d) as viewer:
        last_time = time.time()  # 初始化上一次迭代的时间
        while 1:  
            viewer.sync()  
            error = np.zeros((6, 1))
            sensor_force =client_ft.get_data()-initial_force_data # d.xfrc_applied[links_len-2]/100
            applied_force = np.zeros([6,])
            applied_force[0] = -sensor_force[0][1]/100
            applied_force[1] = -sensor_force[0][0]/100
            applied_force[2] = -sensor_force[0][2]/100
            applied_force[3] = -sensor_force[0][3]
            applied_force[4] = sensor_force[0][4]
            applied_force[5] = -sensor_force[0][5]
            
            print(applied_force)

            wrench_external = applied_force.reshape(-1, 1)
            # wrench_external = np.array([0,0,0,0,0,0])
            #############################################################
            #              Parameter set for Admittance Control         #
            #############################################################

            arm_position_ = np.array(d.xpos[7]).reshape(-1, 1)  # Current arm end effector position
            arm_orientation_, _ = forward_controller(TM5_chain, d.qpos)
            arm_orientation_ = R.from_matrix(arm_orientation_)
            # print("arm_orientation_",arm_orientation_.as_matrix())
                                                                # arm_orientation_ [[ 1.00000000e+00 -2.13589790e-13  3.26794897e-07]
                                                                #  [ 3.26794897e-07  6.53589685e-07 -1.00000000e+00]
                                                                #  [ 1.70730078e-20  1.00000000e+00  6.53589685e-07]]
            arm_desired_twist_adm_, error = admittance_control_fixed(d, error, 
                                                                    arm_position_, 
                                                                    desired_pose_position_, 
                                                                    desired_pose_orientation_, 
                                                                    arm_orientation_, 
                                                                    D, M, K, 
                                                                    arm_desired_twist_adm_, 
                                                                    m.opt.timestep, 
                                                                    wrench_external.reshape(-1, 1))
            # print(error)
            joints = np.zeros(9)
            joints[2:links_len-1] = d.qpos
            All_matrix = TM5_chain.forward_kinematics(joints,True)[2:8]
            J = calculate_jacobian(All_matrix)

            # 增加条件数检查
            # cond_number = np.linalg.cond(J)
            # if cond_number < 1/sys.float_info.epsilon:
            J_pinv = calculate_pseudo_inverse(J)
            joint_velocities = J_pinv @ arm_desired_twist_adm_
            # else:
            #     joint_velocities = np.zeros(J.shape[1])

            # 限制关节速度
            joint_velocities = limit_joint_velocities(joint_velocities, max_velocity=arm_max_vel_)

            # 平滑关节速度（可选）
            # joint_velocities = smooth_velocities(joint_velocities)

            d.ctrl = joint_velocities.reshape([6,])
            # print(d.ctrl)
            mj.mj_step(m, d)
            # time.sleep(0.001)
            # 控制仿真步长与现实时间同步
            time_step = m.opt.timestep  # 仿真的时间步长
            next_time = last_time + time_step
            sleep_time = max(0, next_time - time.time())
            # time.sleep(sleep_time)
            last_time = next_time  # 更新上一次迭代的时间
            count+=1


Mujoco_Thread()