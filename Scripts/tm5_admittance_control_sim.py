import mujoco as mj 
import mujoco.viewer
import numpy as np 
import math
from scipy.spatial.transform import Rotation as R
import ikpy.chain
import time
from math import pi

"""
Mujoco Model Setup
"""
m = mj.MjModel.from_xml_path(r'C:\Users\15738\Desktop\Admittance_control\AR_Project_Admittance_Control\TM5_Description\scene.xml')
m.opt.gravity[0] = 0.0  # x轴重力
m.opt.gravity[1] = 0.0  # y轴重力
m.opt.gravity[2] = 0.0  # z轴重力
d = mj.MjData(m)
m.opt.timestep = 0.01

"""
Ikpy Setup
"""
# End Effector-joint6 to joint3
TM5_Joint = ikpy.chain.Chain.from_urdf_file(r"C:\Users\15738\Desktop\Admittance_control\AR_Project_Admittance_Control\TM5.urdf")

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
    print('error:=',error.reshape(6,))
    print('----------------------------------------------')


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
    links_len = len(chain.links) 
    result = chain.inverse_kinematics(pos,euler,"all")[2:links_len-1]
    return result





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
    for i in range(100):
        d.ctrl = initial_joints
        mj.mj_step(m, d)
    with mj.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            viewer.sync()
            t= t+1
            applied_force = d.xfrc_applied[links_len-2] #read external forces in mujoco 
            wrench_external_ = applied_force.reshape(-1, 1)
            arm_position_ = np.array(d.xpos[links_len-2]).reshape(-1, 1)  # Current arm end effector position
            arm_orientation_ = R.from_quat(d.xquat[links_len-2])  # Current arm end effector orientation
            arm_desired_twist_adm_,linear_disp,angular_disp,error = admittance_control_fixed(d,error, 
                        arm_position_, 
                        desired_pose_position_, 
                        desired_pose_orientation_, 
                        arm_orientation_, 
                        D, M, K, 
                        arm_desired_twist_adm_, m.opt.timestep,
                        wrench_external_)
            print(desired_pose_position_)
            
            current_angular,current_linear= forward_controller(chain,d.qpos[0:links_len-3])
            new_linear = current_linear +linear_disp.reshape([3,])
            fix_angular,fix_linear = forward_controller(chain,initial_pos[0:links_len-3])
            # new_angular = R.as_matrix(desired_pose_orientation_)
            new_angular = fix_angular
            result_check = inverse_controller(chain,new_linear,new_angular)
            #gpt code here, compare result_check and initial_joints , if the rotate |angle| > pi/4,the angle should not exceed the limit. Joint is a 6d vector

            angle_difference = result_check - initial_joints
            angle_difference = np.clip(angle_difference, -np.pi/3, np.pi/3)
            result_after_check = initial_joints + angle_difference


            d.ctrl[0:links_len-3] = result_check
            print(links_len-3)
            mj.mj_step(m, d)
            # time.sleep(0.01)
        
initial_pos = [-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00, 1.56651260e+00, -1.95721289e-12]
def main():
    global joint_pos, joint_ori,initial_pos

    chain = TM5_Joint
    D_Pos = np.array([0.27629562, -0.12396941 , 0.82000005]).reshape([3,1])
    D_Ori = [-5.02137269e-01 ,-5.02137270e-01 ,-4.97853556e-01 ,-4.97853555e-01]
    # print(Joint_Pos):[[ 0.27629562 -0.12396941  0.82000005] ----9
                    # [ 0.16314977 -0.12300002  0.82000005] ----8
                    # [ 0.16314977 -0.12300002  0.71400005] ----7
                    # [-0.20134143  0.0008625   0.52401643]]    ----6
    
    # print(Joint_Ori) :[[-5.02137269e-01 -5.02137270e-01 -4.97853556e-01 -4.97853555e-01]
    #                 [-7.10129337e-01 -3.55775981e-10 -3.55775859e-10 -7.04071250e-01]
    #                 [-7.07105159e-01  7.07105159e-01 -1.51452553e-03  1.51452482e-03]
    #                 [-6.86487028e-01  6.87206886e-01  1.66573393e-01  1.69515663e-01]]
    Control_Loop(chain,initial_pos,D_Pos,D_Ori)

if __name__ == '__main__':
    main()