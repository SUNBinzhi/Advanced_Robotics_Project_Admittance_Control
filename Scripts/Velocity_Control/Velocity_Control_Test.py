
import time
import mujoco as mj
import mujoco.viewer
import numpy as np
import math 
# from visual_kinematics.RobotSerial import *
import numpy as np
from math import pi
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
from scipy.spatial.transform import Rotation as R

# Mujoco Model init
# m = mj.MjModel.from_xml_path('/home/aaron/Desktop/Mujoco_Adimttance_Control/TM5-900/TM5-900_description/TM5/scene.xml')
# m = mj.MjModel.from_xml_path('/home/aaron/Desktop/UR5e_files/onedrive_11_21/TM5/scene.xml')
m = mj.MjModel.from_xml_path('F:\Dropbox\Dropbox\CPII\CPII\Desktop\Mujoco_Adimttance_Control\Github_Version\Mujoco_Admittance_Control_TM5-900\TM5-900\TM5-900_description\TM5\scene.xml')
m.opt.gravity[0] = 0.0  # x轴重力
m.opt.gravity[1] = 0.0  # y轴重力
m.opt.gravity[2] = 0.0  # z轴重力

d = mj.MjData(m)

my_chain = ikpy.chain.Chain.from_urdf_file("F:\Dropbox\Dropbox\CPII\CPII\Desktop\Mujoco_Adimttance_Control\Github_Version\Mujoco_Admittance_Control_TM5-900\TM5-900\script\ikpy_script\Kinematics_Test\TM5.urdf")



#Desired pos and orientation
xyz = np.array([0.05, -0.123, 0.82])

abc = np.array([-pi/2, 0, pi/2])
abc = R.from_euler("xyz",abc)
eye3 = abc.as_matrix()
print('------------------------------------',eye3)
# rot = pi
angles = my_chain.inverse_kinematics(xyz,eye3,"all")[2:8]
print(angles)

def control_loop():
    with mj.viewer.launch_passive(m, d) as viewer:
        target_z = 400.0
        step_size = 0.1
        i = 0
        while viewer.is_running():
            # i+=1
            xyz = np.array([0.05, -0.123, 0.82])
            abc = np.array([-pi/2, pi, -pi/2])
            abc = R.from_euler("xyz",abc)
            eye3 = abc.as_matrix()
            eye3  =np.array([[ 3.26794897e-07,  3.26794789e-07, -1.00000000e+00],
 [-1.00000000e+00 , 3.26794895e-07 ,-3.26794790e-07],
 [ 3.26794789e-07  ,1.00000000e+00 , 3.26794896e-07]])
            # rot = pi
            angles = my_chain.inverse_kinematics(xyz,eye3,"all")[2:8]
            print('------------------------------------',angles)
            step_start = time.time()
            d.ctrl = angles
            mj.mj_step(m, d)
            with viewer.lock():
                viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
            viewer.sync()

            target_z += step_size
            if target_z > 800:
                target_z = 400

            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


        
def main():
    control_loop()

if __name__ == '__main__':
    main()