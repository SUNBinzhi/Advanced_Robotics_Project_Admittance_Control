# import rclpy
# from rclpy.node import Node
# from tm_msgs.srv import SetPositions
# from std_msgs.msg import Float64MultiArray  # 导入消息类型
# import sys
# import time

# class SetPositionsClient(Node):
#     def __init__(self, args=None):
#         super().__init__('set_positions_client')
#         self.client = self.create_client(SetPositions, 'set_positions')
#         self.subscription = self.create_subscription(
#             Float64MultiArray, 
#             'positions_topic', 
#             self.positions_callback, 
#             10)
#         self.positions = None  # 初始化位置变量
#         # while not self.client.wait_for_service(timeout_sec=1.0):
#         #     if not rclpy.ok():
#         #         self.get_logger().error('Interrupted while waiting for the service. Exiting.')
#         #         sys.exit(1)
#         #     self.get_logger().info('Service not available, waiting again...')


#     def positions_callback(self, msg):
#         # self.positions = msg.data  # 更新位置数据
#         # self.get_logger().info(f'Received positions: {self.positions}')
#         # # 发送服务请求并处理响应
#         self.positions = msg.data
#         self.get_logger().info(f'Received positions: {self.positions}')
#         self.send_and_process_request()

#     def send_and_process_request(self):
#         if self.positions is None:
#             self.get_logger().error('No positions received yet.')
#             return

#         motion_type = SetPositions.Request.PTP_J  # 或其他适当的值
#         velocity = 3.14  # rad/s
#         acc_time = 0.0
#         blend_percentage = 100
#         fine_goal = False

#         self.send_request(motion_type, velocity, acc_time, blend_percentage, fine_goal)
#         self.process_response()



#     def send_request(self, motion_type, velocity, acc_time, blend_percentage, fine_goal):
#         if self.positions is None:
#             self.get_logger().error('No positions received yet.')
#             return
#         request = SetPositions.Request()
#         request.motion_type = motion_type
#         request.positions = self.positions
#         request.velocity = velocity
#         request.acc_time = acc_time
#         request.blend_percentage = blend_percentage
#         request.fine_goal = fine_goal
#         self.future = self.client.call_async(request)

#     def process_response(self):
#         # while rclpy.ok():
#         #     rclpy.spin_once(self)
#         #     if self.future.done():
#         #         try:
#         #             response = self.future.result()
#         #             if response.ok:
#         #                 self.get_logger().info('OK')
#         #             else:
#         #                 self.get_logger().info('not OK')
#         #         except Exception as e:
#         #             self.get_logger().error('Service call failed %r' % (e,))
#         #         break
#         if self.future is None:
#             return

#         # rclpy.spin_once(self)
#         if self.future.done():
#             try:
#                 response = self.future.result()
#                 if response.ok:
#                     self.get_logger().info('OK')
#                 else:
#                     self.get_logger().info('not OK')
#             except Exception as e:
#                 self.get_logger().error(f'Service call failed {e}')

#     def shutdown_ros(self):
#         rclpy.shutdown()


# 使用方法示例
# def main(args=None):
#     rclpy.init(args=args)
#     client = SetPositionsClient()
#     # 循环等待位置数据
#     while rclpy.ok():
#         rclpy.spin_once(client)
#     client.shutdown_ros()

    
# if __name__ == '__main__':
#     main()

# # import rclpy
# # from rclpy.node import Node
# # from tm_msgs.srv import SetPositions
# # from std_msgs.msg import Float64MultiArray
# # import sys
# # import time
# # import threading
# # import queue

# # class PositionSubscriber(Node):
# #     def __init__(self, data_queue):
# #         super().__init__('position_subscriber')
# #         self.subscription = self.create_subscription(
# #             Float64MultiArray, 
# #             'positions_topic', 
# #             self.positions_callback, 
# #             10)
# #         self.data_queue = data_queue

# #     def positions_callback(self, msg):
# #         self.data_queue.put(msg.data)
# #         self.get_logger().info(f'Received positions: {msg.data}')

# # class CommandClient(Node):
# #     def __init__(self, data_queue):
# #         super().__init__('command_client')
# #         self.client = self.create_client(SetPositions, 'set_positions')
# #         self.data_queue = data_queue
# #         self.thread = threading.Thread(target=self.process_requests)
# #         self.thread.start()

# #     def process_requests(self):
# #         while rclpy.ok():
# #             try:
# #                 positions = self.data_queue.get(timeout=1)
# #                 self.send_request(positions)
# #                 self.process_response()
# #             except queue.Empty:
# #                 continue

# #     def send_request(self, positions):
# #         motion_type = SetPositions.Request.PTP_J
# #         velocity = 3.14
# #         acc_time = 0.0
# #         blend_percentage = 100
# #         fine_goal = False

# #         request = SetPositions.Request()
# #         request.motion_type = motion_type
# #         request.positions = positions
# #         request.velocity = velocity
# #         request.acc_time = acc_time
# #         request.blend_percentage = blend_percentage
# #         request.fine_goal = fine_goal
# #         self.future = self.client.call_async(request)

# #     def process_response(self):
# #         if self.future is None:
# #             return

# #         if self.future.done():
# #             try:
# #                 response = self.future.result()
# #                 if response.ok:
# #                     self.get_logger().info('OK')
# #                 else:
# #                     self.get_logger().info('not OK')
# #             except Exception as e:
# #                 self.get_logger().error(f'Service call failed {e}')

# # def main(args=None):
# #     rclpy.init(args=args)
# #     data_queue = queue.Queue()
# #     subscriber = PositionSubscriber(data_queue)
# #     client = CommandClient(data_queue)

# #     executor = rclpy.executors.MultiThreadedExecutor()

# #     executor.add_node(subscriber)
# #     executor.add_node(client)

# #     try:
# #         executor.spin()
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         subscriber.destroy_node()
# #         client.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()


# # import rclpy
# # from rclpy.node import Node
# # from tm_msgs.srv import SetPositions
# # from std_msgs.msg import Float64MultiArray
# # import threading
# # import queue

# # class PositionSubscriber(Node):
# #     def __init__(self, data_queue):
# #         super().__init__('position_subscriber')
# #         self.subscription = self.create_subscription(
# #             Float64MultiArray, 
# #             'positions_topic', 
# #             self.positions_callback, 
# #             10)
# #         self.data_queue = data_queue

# #     def positions_callback(self, msg):
# #         with self.data_queue.mutex:
# #             self.data_queue.queue.clear()  # 清空队列以确保只有最新的数据
# #         self.data_queue.put(msg.data)
# #         # self.get_logger().info(f'Received positions: {msg.data}')

# # class CommandClient(Node):
# #     def __init__(self, data_queue):
# #         super().__init__('command_client')
# #         self.client = self.create_client(SetPositions, 'set_positions')
# #         self.data_queue = data_queue
# #         self.thread = threading.Thread(target=self.process_requests)
# #         self.thread.start()

# #     def process_requests(self):
# #         while rclpy.ok():
# #             try:
# #                 # 只处理队列中的最新数据
# #                 positions = self.data_queue.get(timeout=1)
# #                 self.send_request(positions)
# #                 self.process_response()
# #             except queue.Empty:
# #                 continue

# #     def send_request(self, positions):
# #         motion_type = SetPositions.Request.PTP_J
# #         velocity = 3.14
# #         acc_time = 0.0
# #         blend_percentage = 100
# #         fine_goal = False

# #         request = SetPositions.Request()
# #         request.motion_type = motion_type
# #         request.positions = positions
# #         request.velocity = velocity
# #         request.acc_time = acc_time
# #         request.blend_percentage = blend_percentage
# #         request.fine_goal = fine_goal
# #         self.future = self.client.call_async(request)
# #         self.get_logger().info(f'Client Send positions: {positions}')

# #     def process_response(self):
# #         if self.future is None:
# #             return

# #         if self.future.done():
# #             try:
# #                 response = self.future.result()
# #                 if response.ok:
# #                     self.get_logger().info('OK')
# #                 else:
# #                     self.get_logger().info('not OK')
# #             except Exception as e:
# #                 self.get_logger().error(f'Service call failed {e}')

# # def main(args=None):
# #     rclpy.init(args=args)
# #     data_queue = queue.Queue()
# #     subscriber = PositionSubscriber(data_queue)
# #     client = CommandClient(data_queue)

# #     executor = rclpy.executors.MultiThreadedExecutor()
# #     executor.add_node(subscriber)
# #     executor.add_node(client)

# #     try:
# #         executor.spin()
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         subscriber.destroy_node()
# #         client.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()
# # import rclpy
# # from rclpy.node import Node
# # from techman_robot_msgs.srv import TechmanRobotCommand
# # from std_msgs.msg import Float64MultiArray
# # import threading
# # import queue
# # import math

# # class PositionSubscriber(Node):
# #     def __init__(self, data_queue):
# #         super().__init__('position_subscriber')
# #         self.subscription = self.create_subscription(
# #             Float64MultiArray, 
# #             'positions_topic', 
# #             self.positions_callback, 
# #             10)
# #         self.data_queue = data_queue

# #     def positions_callback(self, msg):
# #         with self.data_queue.mutex:
# #             self.data_queue.queue.clear()  # 清空队列以确保只有最新的数据
# #         self.data_queue.put(msg.data)

# # class CommandClient(Node):
# #     def __init__(self, data_queue):
# #         super().__init__('techman_command_client')
# #         self.client = self.create_client(TechmanRobotCommand, 'tm_send_command')
# #         self.data_queue = data_queue
# #         self.thread = threading.Thread(target=self.process_requests)
# #         self.thread.start()

# #     def process_requests(self):
# #         while rclpy.ok():
# #             try:
# #                 positions = self.data_queue.get(timeout=1)
# #                 self.send_command(positions)
# #                 self.process_response()
# #             except queue.Empty:
# #                 continue

# #     def send_command(self, positions):
# #         positions = [math.degrees(pos) for pos in positions]
# #         command = "MOVE_JOG"
# #         command_parameter_string = ','.join(map(str, positions))

# #         request = TechmanRobotCommand.Request()
# #         request.command = command
# #         request.command_parameter_string = command_parameter_string
# #         self.future = self.client.call_async(request)
# #         self.get_logger().info(f'Client Send command: {command}, parameters: {command_parameter_string}')

# #     def process_response(self):
# #         if self.future is None:
# #             return

# #         if self.future.done():
# #             try:
# #                 response = self.future.result()
# #                 if response.is_success:
# #                     self.get_logger().info('Command success')
# #                 else:
# #                     self.get_logger().info('Command failed')
# #             except Exception as e:
# #                 self.get_logger().error(f'Service call failed {e}')

# # def main(args=None):
# #     rclpy.init(args=args)
# #     data_queue = queue.Queue()
# #     subscriber = PositionSubscriber(data_queue)
# #     client = CommandClient(data_queue)

# #     executor = rclpy.executors.MultiThreadedExecutor()
# #     executor.add_node(subscriber)
# #     executor.add_node(client)

# #     try:
# #         executor.spin()
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         subscriber.destroy_node()
# #         client.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

import rclpy
from rclpy.node import Node
from tm_msgs.srv import SendScript
from std_msgs.msg import Float64MultiArray
import sys
import math  # for radian to degree conversion
import time

class SendScriptClient(Node):
    def __init__(self):
        super().__init__('send_script_client')
        self.client = self.create_client(SendScript, 'send_script')
        self.subscription = self.create_subscription(
            Float64MultiArray, 
            'positions_topic', 
            self.positions_callback, 
            10)
        # self.positions = list([0,0,0,0,0,0])
        
        # self.positions = [0,0,0,0,0,0]
        self.positions = None

        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(1)
            self.get_logger().info('Waiting for service...')

    def positions_callback(self, msg):
        self.positions = msg.data
        self.get_logger().info(f'Received positions (radians): {self.positions}')
        self.send_and_process_request()

    def send_and_process_request(self):
        if self.positions is None:
            self.get_logger().error('No positions received yet.')
            return

        # Convert positions from radians to a script command
        command = self.convert_positions_to_script(self.positions)
        self.send_request(command)
        # time.sleep(0.5)
        # command = 'StopAndClearBuffer()'
        # self.send_request(command)

    def send_request(self, command):
        request = SendScript.Request()
        request.id = 'demo'
        # request.script = 'StopAndClearBuffer()'
        # self.future = self.client.call_async(request)
        # time.sleep(0.3)
        request.script = command
        self.future = self.client.call_async(request)
        # request.script = 'StopAndClearBuffer()'
        # time.sleep(0.1)
        # request.script = 'StopAndClearBuffer()'
        # QueueTag(1)

    def convert_positions_to_script(self, positions):
        # Convert radians to degrees and format as a script command
        # positions_deg = [math.degrees(pos) for pos in positions]
        position_str = ','.join(map(str, positions))
        return f"PTP(\"CPP\",{position_str},10,200,0,false,0,2,4)"

    def process_response(self):
        if self.future is None:
            return

        if self.future.done():
            try:
                response = self.future.result()
                if response.ok:
                    self.get_logger().info('Command success')
                else:
                    self.get_logger().info('Command failed')
            except Exception as e:
                self.get_logger().error(f'Service call failed {e}')

def main(args=None):
    rclpy.init(args=args)
    client = SendScriptClient()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            client.process_response()
            # client.future = None  # 重置 future 对象
            # break

    rclpy.shutdown()

if __name__ == '__main__':
    main()

