import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
sys.path.append('/home/aaron/.local/lib/python3.11/site-packages/mujoco')
from scipy.spatial.transform import Rotation as R

# /usr/bin/pip install visual_kinematics
import ikpy.chain
import time
import mujoco as mj
import mujoco.viewer
from math import pi
from visual_kinematics.RobotSerial import *
# from Joint_Pub import PositionsPublisher

# Mujoco Model init
# m = mj.MjModel.from_xml_path('/home/aaron/Desktop/Mujoco_Adimttance_Control/TM5-900/TM5-900_description/TM5/scene.xml')
m = mj.MjModel.from_xml_path('/home/aaron/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/TM5-900_description/TM5/scene.xml')

m.opt.gravity[0] = 0.0  # x轴重力
m.opt.gravity[1] = 0.0  # y轴重力
m.opt.gravity[2] = 0.0  # z轴重力
d = mj.MjData(m)

# #TM500
# dh_params = np.array([[0.1452,      0,          0.5*pi,     0.      ],
#                         [0,         -0.429,     0,          -0.5*pi ],
#                         [0,         -0.4115,    0,          0.      ],
#                         [0.1223,    0,          0.5 * pi,   -0.5*pi ],
#                         [0.106,     0,          -0.5 * pi,  0       ],
#                         [0.11315,   0,          0,          0.      ]
#                         ])
# robot = RobotSerial(dh_params)

# theta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# f = robot.forward(theta)
TM5_chain = ikpy.chain.Chain.from_urdf_file("/home/aaron/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/ikpy_script/Kinematics_Test/TM5.urdf")





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

angle,linear = forward_controller(TM5_chain,[0.,pi/3,-pi/2,pi/6,-pi/2,0.])
angles = np.array([0,pi/3,-pi/2,pi/6,-pi/2,0])
def main():
    with mj.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            step_start = time.time()
            # for i in range(6):       
            #     d.ctrl[i] = theta[i]
            print('real position:')
            print('real position:',d.xpos[7])
            #     # print("-------forward-------")
            #     # print("end frame t_4_4:")
            #     # print(f.t_4_4)
            print("end frame xyz:")
            print(linear)
            #     # print("end frame abc:")
            #     # print(f.euler_3)
            # d.ctrl = [0,pi/3,-pi/2,pi/6,-pi/2,0]
            angles = d.ctrl
            positions_publisher.publish_positions(list(angles))
            time.sleep(0.002)

            mj.mj_step(m, d)
            with viewer.lock():
                viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
            viewer.sync()
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == '__main__':
    main()










