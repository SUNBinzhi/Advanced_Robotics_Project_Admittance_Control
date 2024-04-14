import time
import mujoco as mj
import mujoco.viewer
import numpy as np
import math 
import numpy as np
from math import pi
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
from scipy.spatial.transform import Rotation as R
from math import sin,cos

m = mj.MjModel.from_xml_path(r'C:\Users\15738\Desktop\AR_project\Advanced_Robotics_Project\TM5_Description\scene.xml')
m.opt.gravity[0] = 0.0  # x轴重力
m.opt.gravity[1] = 0.0  # y轴重力
m.opt.gravity[2] = 0.0  # z轴重力
d = mj.MjData(m)

TM5_chain = ikpy.chain.Chain.from_urdf_file(r"C:\Users\15738\Desktop\AR_project\Advanced_Robotics_Project\Scripts\TM5.urdf")

def forward_controller(chain,angles):
        # Ensure the matrix is a NumPy array
        T = chain.forward_kinematics([0,0,angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],0])
        T = np.array(T)
        # Extract the rotation matrix and translation vector
        rotation_matrix = T[:3, :3]
        translation_vector = T[:3, 3]
        xyz_coordinates = translation_vector
        return rotation_matrix, xyz_coordinates


joint_angles = [-4.28372736e-03, -4.88533739e-01, 1.57943555e+00, -1.09090181e+00, 1.56651260e+00, -1.95721289e-12]

t = 0
with mj.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        viewer.sync()
        rotation,xyz = forward_controller(TM5_chain,joint_angles)
        quat = R.from_matrix(rotation).as_quat()
        d.ctrl = joint_angles
        mj.mj_step(m, d)
        t = t+1
        if t>=1000:
             break
print('current pos xyz = ',xyz)
print('current quat by ikpy = ',quat)
print('current quat by mujoco = ',d.xquat[7])
