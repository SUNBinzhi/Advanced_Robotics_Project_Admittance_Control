import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetPositions
import sys

class SetPositionsClient(Node):
    def __init__(self):
        super().__init__('set_positions_client')
        self.client = self.create_client(SetPositions, 'set_positions')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(1)
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, motion_type, positions, velocity, acc_time, blend_percentage, fine_goal):
        request = SetPositions.Request()
        request.motion_type = motion_type
        request.positions = positions
        request.velocity = velocity
        request.acc_time = acc_time
        request.blend_percentage = blend_percentage
        request.fine_goal = fine_goal
        self.future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    set_positions_client = SetPositionsClient()
    motion_type = SetPositions.Request.PTP_J
    positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    velocity = 0.4  # rad/s
    acc_time = 0.2
    blend_percentage = 10
    fine_goal = False
    set_positions_client.send_request(motion_type, positions, velocity, acc_time, blend_percentage, fine_goal)

    while rclpy.ok():
        rclpy.spin_once(set_positions_client)
        if set_positions_client.future.done():
            try:
                response = set_positions_client.future.result()
                if response.ok:
                    set_positions_client.get_logger().info('OK')
                else:
                    set_positions_client.get_logger().info('not OK')
            except Exception as e:
                set_positions_client.get_logger().error(
                    'Service call failed %r' % (e,))
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
