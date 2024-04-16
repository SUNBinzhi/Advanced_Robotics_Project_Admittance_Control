import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetEvent
import sys

class SetEventClient(Node):
    def __init__(self):
        super().__init__('set_event_client')
        self.client = self.create_client(SetEvent, 'set_event')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(1)
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, func, arg0, arg1):
        request = SetEvent.Request()
        request.func = func
        request.arg0 = arg0
        request.arg1 = arg1
        self.future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    set_event_client = SetEventClient()
    set_event_client.send_request(SetEvent.Request.STOP, 0, 0)

    while rclpy.ok():
        rclpy.spin_once(set_event_client)
        if set_event_client.future.done():
            try:
                response = set_event_client.future.result()
                if response.ok:
                    set_event_client.get_logger().info('OK')
                else:
                    set_event_client.get_logger().info('not OK')
            except Exception as e:
                set_event_client.get_logger().error(
                    'Service call failed %r' % (e,))
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
