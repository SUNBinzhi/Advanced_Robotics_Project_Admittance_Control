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
import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetPositions
from std_msgs.msg import Float64MultiArray
import threading
import queue
# with mj.viewer.launch_passive(m, d) as viewer:

#TM500 D-H Params
# dh_params = np.array([[0.1452,      0,          0.5*pi,     0.      ],
#                         [0,         -0.429,     0,          -0.5*pi ],
#                         [0,         -0.4115,    0,          0.      ],
#                         [0.1223,    0,          0.5 * pi,   -0.5*pi ],
#                         [0.106,     0,          -0.5 * pi,  0       ],
#                         [0.11315,   0,          0,          0.      ]
#                         ])
# robot = RobotSerial(dh_params)
TM5_chain = ikpy.chain.Chain.from_urdf_file("/home/aaron/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/ikpy_script/Kinematics_Test/TM5.urdf")

# Basic Parameters for the End-Effector
M_param = 1e1
M = np.diag([M_param,M_param,M_param,M_param,M_param,M_param])

d_param =50 #50
d_param_rot =10 #50
D = np.diag([d_param, d_param, d_param, d_param_rot, d_param_rot, d_param_rot])
k_param = 1000 #100
k_param_rot = 20#100
K = np.diag([k_param, k_param, k_param, k_param_rot, k_param_rot, k_param_rot])
arm_max_acc_ = 100.0  # Maximum acceleration
arm_max_vel_ = 100.0  # Maximum velocity

m = mj.MjModel.from_xml_path('/home/aaron/Desktop/Mujoco_Adimttance_Control/Github_Version/Mujoco_Admittance_Control_TM5-900/TM5-900/TM5-900_description/TM5/scene.xml')
m.opt.gravity[0] = 0.0  # x轴重力
m.opt.gravity[1] = 0.0  # y轴重力
m.opt.gravity[2] = 0.0  # z轴重力
d = mj.MjData(m)



alpha = np.zeros([6,])
wrench_external = np.zeros(6)









# Admittance control function with fixed error shape assignment
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


def Mujoco_Thread(data_queue):
    global alpha
    global wrench_external
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
        data_queue.put(alpha)
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
            arm_orientation_,nothing = forward_controller(TM5_chain,d.qpos)
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
            current_angular,current_linear= forward_controller(TM5_chain,d.qpos)
            new_linear = current_linear +linear_disp.reshape([3,])
            # new_angular = current_angular-angular_disp.reshape([3,])
            new_angular =  np.array([[ 3.26794897e-07,  3.26794789e-07, -1.00000000e+00],
                                        [-1.00000000e+00 , 3.26794895e-07 ,-3.26794790e-07],
                                        [ 3.26794789e-07  ,1.00000000e+00 , 3.26794896e-07]])

            d.ctrl = inverse_controller(new_linear,new_angular)
            alpha = d.ctrl
            data_queue.put(alpha)
            mj.mj_step(m, d)

            time.sleep(0.01)



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
                    
                    wrench_external = np.array([0, (data_values_1[1]+data_values_2[1]+data_values_3[1]+data_values_4[1])/20, (data_values_1[2]+data_values_2[2]+data_values_3[2]+data_values_4[2])/20, 0, 0, 0])
                    # print(wrench_external)
    finally:
        ser.close()


class CommandClient(Node):
    def __init__(self, data_queue):
        super().__init__('command_client')
        self.client = self.create_client(SetPositions, 'set_positions')
        self.data_queue = data_queue
        self.thread = threading.Thread(target=self.process_requests)
        self.thread.start()

    def process_requests(self):
        while rclpy.ok():
            try:
                # 检查队列中是否有数据
                if not self.data_queue.empty():
                    # 清空队列，只保留最新的数据
                    with self.data_queue.mutex:
                        self.data_queue.queue.clear()
                    positions = self.data_queue.get(timeout=1)
                    positions = [float(pos) for pos in positions]
                    self.send_request(positions)
                    self.process_response()
                else:
                    time.sleep(0.1)  # 队列为空时，稍微等待一下再次检查
            except queue.Empty:
                continue

    def send_request(self, positions):
        motion_type = SetPositions.Request.PTP_J
        velocity = 3.14
        acc_time = 0.0
        blend_percentage = 100
        fine_goal = False

        request = SetPositions.Request()
        request.motion_type = motion_type
        request.positions = positions
        request.velocity = velocity
        request.acc_time = acc_time
        request.blend_percentage = blend_percentage
        request.fine_goal = fine_goal
        self.future = self.client.call_async(request)

    def process_response(self):
        if self.future is None:
            return

        if self.future.done():
            try:
                response = self.future.result()
                if response.ok:
                    self.get_logger().info('OK')
                else:
                    self.get_logger().info('not OK')
            except Exception as e:
                self.get_logger().error(f'Service call failed {e}')

# rclpy.init(args=None)
# positions_publisher = PositionsPublisher()
def main():
    rclpy.init(args=None)

    # 创建数据队列
    data_queue = queue.Queue()

    # 实例化 CommandClient
    command_client = CommandClient(data_queue)

    # 创建并启动线程
    thread1 = threading.Thread(target=Mujoco_Thread, args=(data_queue,))
    thread2 = threading.Thread(target=command_client.process_requests, args=())
    thread3 = threading.Thread(target=serial_read_thread, args=("/dev/ttyUSB0", 115200))

    # 启动线程
    thread1.start()
    thread2.start()
    thread3.start()

    # 等待线程完成
    thread1.join()
    thread2.join()
    thread3.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
