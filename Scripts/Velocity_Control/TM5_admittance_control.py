import time
import mujoco as mj
import mujoco.viewer
import numpy as np
import math as m
import numpy as np
from scipy.spatial.transform import Rotation as R

# from visual_kinematics.RobotSerial import *
#TM500 D-H Params
# dh_params = np.array([[0.1452,      0,          0.5*pi,     0.      ],
#                         [0,         -0.429,     0,          -0.5*pi ],
#                         [0,         -0.4115,    0,          0.      ],
#                         [0.1223,    0,          0.5 * pi,   -0.5*pi ],
#                         [0.106,     0,          -0.5 * pi,  0       ],
#                         [0.11315,   0,          0,          0.      ]
#                         ])

# robot = RobotSerial(dh_params)

# Basic Parameters for the End-Effector
# M = np.diag([1.576, 1.576, 1.576, 1.576, 1.576, 1.576])
M_param = 1e1
M = np.diag([M_param,M_param,M_param,M_param,M_param,M_param])

d_param =100 #50
d_param_rot =10 #50
D = np.diag([d_param, d_param, d_param, d_param_rot, d_param_rot, d_param_rot])
k_param = 800 #100
k_param_rot = 20#100
K = np.diag([k_param, k_param, k_param, k_param_rot, k_param_rot, k_param_rot])
arm_max_acc_ = 100.0  # Maximum acceleration
arm_max_vel_ = 10.0  # Maximum velocity



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
    

    #Admittance control's calcuation result send
    # desired_pose_position_, desired_pose_orientation_= admittance_control_send(d,linear_disp,angular_disp)

    #Update the desired velocity, position and orientation of the end-effector
    # print('desired_velocity = ',arm_desired_twist_adm_.reshape(6,))
    # print('----------------------------------------------')
    # print('***********************************************************************************')

    return arm_desired_twist_adm_,linear_disp,angular_disp,error



    



