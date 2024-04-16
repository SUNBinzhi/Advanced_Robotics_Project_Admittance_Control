import rclpy
from rclpy.node import Node
from techman_robot_msgs.srv import TechmanRobotCommand
import sys

class TechmanRobotCommandClient(Node):

    def __init__(self):
        super().__init__('send_command_client')
        self.client = self.create_client(TechmanRobotCommand, 'tm_send_command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Client interrupted while waiting for service to appear.')
                sys.exit(1)
            self.get_logger().info('Waiting for service...')

    def send_command(self, command, command_parameter_string):
        request = TechmanRobotCommand.Request()
        request.command = command
        request.command_parameter_string = command_parameter_string
        self.future = self.client.call_async(request)
        return self.future

def main(args=None):
    rclpy.init(args=args)

    command_client = TechmanRobotCommandClient()
    future = command_client.send_command("MOVE_JOG", "0,0,0,0,0,0")

    while rclpy.ok():
        rclpy.spin_once(command_client)
        if future.done():
            try:
                response = future.result()
                if response.is_success:
                    command_client.get_logger().info('Command success')
                else:
                    command_client.get_logger().info('Command failed')
            except Exception as e:
                command_client.get_logger().error(
                    'Service call failed %r' % (e,))
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
