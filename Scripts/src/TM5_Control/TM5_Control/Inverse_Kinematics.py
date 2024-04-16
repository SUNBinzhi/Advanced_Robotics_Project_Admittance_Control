import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
sys.path.append('/home/aaron/.local/lib/python3.11/site-packages/mujoco')

# /usr/bin/pip install visual_kinematics
import ikpy.chain
import numpy as np
import time
import mujoco as mj
import mujoco.viewer
from math import pi
# from visual_kinematics.RobotSerial import *
# from Joint_Pub import PositionsPublisher

# Mujoco Model init
# m = mj.MjModel.from_xml_path('/home/aaron/Desktop/Mujoco_Adimttance_Control/TM5-900/TM5-900_description/TM5/scene.xml')
m = mj.MjModel.from_xml_path("/home/aaron/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/TM5-900_description/TM5/scene.xml")
m.opt.gravity[0] = 0.0  # x轴重力
m.opt.gravity[1] = 0.0  # y轴重力
m.opt.gravity[2] = 0.0  # z轴重力
d = mj.MjData(m)
my_chain = ikpy.chain.Chain.from_urdf_file("/home/aaron/Gail_Cons/Utils/TM5.urdf")

initial_angles = np.array([-4.28372736e-03 ,-4.88533739e-01,  1.57943555e+00, -1.09090181e+00,
                                        -1.56651260e+00 ,-1.95721289e-12])
initial_pos = np.array([ 0.05 , -0.123 , 0.82 ])

def inverse_controller(pos,euler):
    if euler is None:
        euler = np.array([[ 3.26794897e-07,  3.26794789e-07, -1.00000000e+00],
                        [-1.00000000e+00 , 3.26794895e-07 ,-3.26794790e-07],
                        [ 3.26794789e-07  ,1.00000000e+00 , 3.26794896e-07]])
    result = my_chain.inverse_kinematics(pos,euler,"all")[2:8]
    return result

theta = inverse_controller(initial_pos,None)

class PositionsPublisher(Node):
    def __init__(self):
        super().__init__('positions_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'positions_topic', 10)

    def publish_positions(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')





rclpy.init(args=None)
positions_publisher = PositionsPublisher()
m.opt.timestep = 0.002

def control_send(target_angles):
    for i in range(6):
        d.ctrl[i] = target_angles[i]

def main():
    with mj.viewer.launch_passive(m, d) as viewer:
        target_z = 400.0
        step_size = 0.1

        while viewer.is_running():
            step_start = time.time()
            # for i in range(6):       
            #     d.ctrl[i] = theta[i]
            # print('real position:')
            # print('real position:',d.xpos[7])
            #     # print("-------forward-------")
            #     # print("end frame t_4_4:")
            #     # print(f.t_4_4)
            # print("end frame xyz:")
            # print(f.t_3_1.reshape([3, ]))
            #     # print("end frame abc:")
            #     # print(f.euler_3)
            # angles = d.ctrl
            control_send(theta)
            positions_publisher.publish_positions(list(theta))
            time.sleep(0.002)

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


if __name__ == '__main__':
    main()









