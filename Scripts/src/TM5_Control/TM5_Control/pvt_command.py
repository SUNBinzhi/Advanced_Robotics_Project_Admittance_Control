# import rclpy
# from rclpy.node import Node
# from tm_msgs.srv import SendScript
# from std_msgs.msg import Float64MultiArray
# import sys

# class SendScriptClient(Node):
#     def __init__(self):
#         super().__init__('send_script_client')
#         self.client = self.create_client(SendScript, 'send_script')
#         self.subscription = self.create_subscription(
#             Float64MultiArray,
#             'positions_topic',
#             self.positions_callback,
#             10)
#         self.positions = None

#         while not self.client.wait_for_service(timeout_sec=1.0):
#             if not rclpy.ok():
#                 self.get_logger().error('Interrupted while waiting for the service. Exiting.')
#                 sys.exit(1)
#             self.get_logger().info('service not available, waiting again...')

#     def positions_callback(self, msg):
#         # Expecting msg.data to contain [X, Y, Z, RX, RY, RZ, duration]
#         if len(msg.data) < 7:
#             self.get_logger().error('Received positions array does not contain enough elements.')
#             return
#         self.positions = msg.data[:6]  # Extract positions
#         self.duration = msg.data[6]  # Extract duration
#         self.get_logger().info(f'Received positions: {self.positions}, duration: {self.duration}')
#         self.send_pvt_command()

#     def send_pvt_command(self):
#         if self.positions is None or self.duration is None:
#             self.get_logger().error('No positions or duration received yet.')
#             return
        
#         # 构建PVT命令
#         command = self.construct_pvt_script(self.positions, self.duration)
#         self.send_request(command)

#     def send_request(self, command):
#         request = SendScript.Request()
#         request.id = 'demo'
#         request.script = command
#         self.future = self.client.call_async(request)

#     def construct_pvt_script(self, positions, duration):
#         # 开始PVT模式，使用笛卡尔坐标
#         script = "PVTEnter(1)\n"
        
#         # 构建PVT命令，速度都设为10
#         velocity = 10  # 设置速度值
#         # 格式化位置和速度
#         vel = [0.0,0.0,100.0,0.0,0.0,0.0]
#         positions_str = ','.join(map(str, positions))  # XYZRXRYRZ
#         # velocities_str = ','.join(map(str, vel))  # 将所有速度都设置为10
#         velocities_str = ','.join(['0.0'] * 6)  # 将所有速度都设置为10
        
#         # 添加PVT点
#         script += f"PVTPoint({positions_str},{velocities_str},{duration})\n"
        
#         # 结束PVT模式
#         # script += "PVTExit()\n"
        
#         return script

#     def process_response(self):
#         if self.future is None:
#             return

#         if self.future.done():
#             try:
#                 response = self.future.result()
#                 if response.ok:
#                     self.get_logger().info('Command sent successfully')
#                 else:
#                     self.get_logger().info('Failed to send command')
#             except Exception as e:
#                 self.get_logger().error(f'Service call failed {e}')

# def main(args=None):
#     rclpy.init(args=args)
#     client = SendScriptClient()

#     while rclpy.ok():
#         rclpy.spin_once(client)
#         if client.future and client.future.done():
#             client.process_response()

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from tm_msgs.srv import SendScript
from std_msgs.msg import Float64MultiArray
import sys

class SendScriptClient(Node):
    def __init__(self):
        super().__init__('send_script_client')
        self.client = self.create_client(SendScript, 'send_script')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'positions_topic',
            self.positions_callback,
            10)
        self.positions = None
        self.duration = None
        self.velocities = None  # 初始化速度列表

        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(1)
            self.get_logger().info('service not available, waiting again...')

    def positions_callback(self, msg):
        # Expecting msg.data to contain [X, Y, Z, RX, RY, RZ, duration, VX, VY, VZ, VRX, VRY, VRZ]
        if len(msg.data) < 13:
            self.get_logger().error('Received positions array does not contain enough elements.')
            return
        self.positions = msg.data[:6]  # Extract positions
        self.duration = msg.data[6]  # Extract duration
        self.velocities = msg.data[7:13]  # Extract velocities
        self.get_logger().info(f'Received positions: {self.positions}, duration: {self.duration}, velocities: {self.velocities}')
        self.send_pvt_command()

    def send_pvt_command(self):
        if self.positions is None or self.duration is None or self.velocities is None:
            self.get_logger().error('No positions, duration, or velocities received yet.')
            return
        
        # 构建PVT命令
        command = self.construct_pvt_script(self.positions, self.duration, self.velocities)
        self.send_request(command)

    def send_request(self, command):
        request = SendScript.Request()
        request.id = 'demo'
        request.script = command
        self.future = self.client.call_async(request)

    def construct_pvt_script(self, positions, duration, velocities):
        # 开始PVT模式，使用笛卡尔坐标
        script = "PVTEnter(1)\n"
        
        # 格式化位置和速度
        positions_str = ','.join(map(str, positions))  # XYZRXRYRZ
        velocities_str = ','.join(map(str, velocities))  # 独立的速度值
        
        # 添加PVT点
        script += f"PVTPoint({positions_str},{velocities_str},{duration})\n"
        
        # 结束PVT模式
        script += "PVTExit()\n"
        
        return script

    def process_response(self):
        if self.future is None:
            return

        if self.future.done():
            try:
                response = self.future.result()
                if response.ok:
                    self.get_logger().info('Command sent successfully')
                else:
                    self.get_logger().info('Failed to send command')
            except Exception as e:
                self.get_logger().error(f'Service call failed {e}')

def main(args=None):
    rclpy.init(args=args)
    client = SendScriptClient()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future and client.future.done():
            client.process_response()

    rclpy.shutdown()