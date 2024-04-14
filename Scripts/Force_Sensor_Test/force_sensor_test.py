import mujoco as mj 
import mujoco.viewer
import numpy as np 
import math
from scipy.spatial.transform import Rotation as R
import ikpy.chain
import time
from math import pi
from math import cos,sin
#Sensor imports
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import numpy as np
import time
import threading

"""
Mujoco Model Setup
"""
m = mj.MjModel.from_xml_path(r'C:\Users\15738\Desktop\AR_project\Advanced_Robotics_Project-1\TM5_Description\scene.xml')
m.opt.gravity[0] = 0.0  # x轴重力
m.opt.gravity[1] = 0.0  # y轴重力
m.opt.gravity[2] = 0.0  # z轴重力
d = mj.MjData(m)
m.opt.timestep = 0.01

"""
Ikpy Setup
"""
# End Effector-joint6 to joint3
TM5_Joint = ikpy.chain.Chain.from_urdf_file(r"C:\Users\15738\Desktop\AR_project\Advanced_Robotics_Project-1\Scripts\TM5.urdf")

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
k_param = 1500 #100
k_param_rot = 20#100
K = np.diag([k_param, k_param, k_param, k_param_rot, k_param_rot, k_param_rot])

# Velocity and Acceleration Limit
arm_max_acc_ = 100.0  # Maximum acceleration
arm_max_vel_ = 10.0  # Maximum velocity

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
    quat_rot_err = arm_orientation_ * desired_pose_orientation_.inv()
    quat_rot_err_normalized = R.from_quat(quat_rot_err.as_quat() / np.linalg.norm(quat_rot_err.as_quat()))
    error[3:] = quat_rot_err_normalized.as_rotvec().reshape(3, 1)
    error[3:] = np.around( error[3:], decimals=3)
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
IK Controller Function
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
    global initial_pos
    links_len = len(chain.links) 
    result = chain.inverse_kinematics(pos,euler,orientation_mode="all",optimizer='least_squares')[2:links_len-1]
    return result


def update_orientation(current_orientation, angular_velocity, dt):
    # Ensure angular_velocity is a 1D array with shape (3,)
    angular_velocity = angular_velocity.flatten()  # This changes shape from (3, 1) to (3,)
    
    # Create a rotation from the angular velocity (assuming small angle approximation)
    delta_rotation = R.from_rotvec(angular_velocity * dt)
    
    # Update the current orientation by multiplying the quaternion
    new_orientation = current_orientation * delta_rotation
    return new_orientation

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

"""
Control Loop Funtion
"""
def Control_Loop(chain,initial_joints,D_Pos,D_Ori):
    # Initial Parameters
    t = 0
    links_len = len(chain.links)
    error = np.zeros((6, 1))
    arm_desired_twist_adm_ = np.zeros((6, 1))
    desired_pose_position_ = D_Pos
    desired_pose_orientation_ = R.from_quat(D_Ori)
    """Sensor"""
    client_ft = ModbusRobotIQ(method="rtu", port="COM3", stopbits=1, bytesize=8, parity='N', baudrate=19200)
    time.sleep(2)
    initial_force_data = client_ft.get_data()
    for i in range(100):
        d.ctrl = initial_joints
        mj.mj_step(m, d)
    with mj.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            viewer.sync()
            t= t+1
            #---force by mouse
            applied_force = d.xfrc_applied[links_len-2] #read external forces in mujoco 

            #force by program
            # d.xfrc_applied[links_len-2] = [0,0,100*sin(t/100),0,0,0]
            # applied_force =  d.xfrc_applied[links_len-2]
            
            #---force by sensor
            # d.xfrc_applied[links_len-2] = client_ft.get_data()-initial_force_data
            applied_force =client_ft.get_data()-initial_force_data # d.xfrc_applied[links_len-2]/100
            print( d.xfrc_applied[links_len-2])

            wrench_external_ = applied_force.reshape(-1, 1)
            arm_position_ = np.array(d.xpos[links_len-2]).reshape(-1, 1)  # Current arm end effector position
            arm_orientation_ = R.from_quat(d.xquat[links_len-2])  # Current arm end effector orientation mujoco

            current_angular,current_linear= forward_controller(chain,d.qpos[0:links_len-3])
            arm_orientation_ = R.from_matrix(current_angular)  # Current arm end effector orientation ikpy
            arm_desired_twist_adm_,linear_disp,angular_disp,error = admittance_control_fixed(d,error, 
                        arm_position_, 
                        desired_pose_position_, 
                        desired_pose_orientation_, 
                        arm_orientation_, 
                        D, M, K, 
                        arm_desired_twist_adm_, m.opt.timestep,
                        wrench_external_)
            # print(desired_pose_position_)
            
            #linear update
            current_angular,current_linear= forward_controller(chain,d.qpos[0:links_len-3])
            new_linear = current_linear +linear_disp.reshape([3,])

            #angular update
            #fix version
            fix_angular,fix_linear = forward_controller(chain,initial_pos[0:links_len-3])
            new_angular = fix_angular

            #none-fixed angular update
            # new_angular = R.from_matrix(current_angular).as_euler('xyz')+angular_disp
            # new_orientation  = update_orientation(R.from_matrix(current_angular), arm_desired_twist_adm_[3:], m.opt.timestep)
            # new_angular = new_orientation.as_euler('xyz')
            # print('666',new_angular)

            result_check = inverse_controller(chain,new_linear,new_angular)
            angle_difference = result_check - initial_joints
            angle_difference = np.clip(angle_difference, -np.pi/3, np.pi/3)
            result_after_check = initial_joints + angle_difference


            d.ctrl[0:links_len-3] = result_check
            # print(links_len-3)
            mj.mj_step(m, d)
            # time.sleep(0.01)
        
initial_pos = [-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00+pi/2, 1.56651260e+00, -1.95721289e-12]
def main():
    global joint_pos, joint_ori,initial_pos

    chain = TM5_Joint
    D_Pos = np.array([ 0.26914695, -0.1239387,   0.60085104]).reshape([3,1])
    D_Ori = [ 0.70861783,  0.70558925, -0.00151765,  0.00151139]

    Control_Loop(chain,initial_pos,D_Pos,D_Ori)

if __name__ == '__main__':
    main()