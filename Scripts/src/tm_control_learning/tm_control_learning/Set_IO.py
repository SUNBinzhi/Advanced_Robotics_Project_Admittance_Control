import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetIO
import sys

class SetIOClient(Node):
    def __init__(self):
        super().__init__('set_io_client')
        self.client = self.create_client(SetIO, 'set_io')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(1)
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, module, type, pin, state):
        request = SetIO.Request()
        request.module = module
        request.type = type
        request.pin = pin
        request.state = state
        self.future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    set_io_client = SetIOClient()
    module = SetIO.Request.MODULE_CONTROLBOX
    type = SetIO.Request.TYPE_DIGITAL_OUT
    pin = 0
    state = SetIO.Request.STATE_ON
    set_io_client.send_request(module, type, pin, state)

    while rclpy.ok():
        rclpy.spin_once(set_io_client)
        if set_io_client.future.done():
            try:
                response = set_io_client.future.result()
                if response.ok:
                    set_io_client.get_logger().info('OK')
                else:
                    set_io_client.get_logger().info('not OK')
            except Exception as e:
                set_io_client.get_logger().error(
                    'Service call failed %r' % (e,))
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
