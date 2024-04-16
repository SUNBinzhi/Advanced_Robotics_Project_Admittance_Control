import rclpy
from rclpy.node import Node
from tm_msgs.srv import AskItem
import sys

class AskItemClient(Node):
    def __init__(self):
        super().__init__('ask_item_client')
        self.client = self.create_client(AskItem, 'ask_item')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(1)
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, item):
        request = AskItem.Request()
        request.id = 'demo'
        request.item = item
        request.wait_time = 1
        self.future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    ask_item_client = AskItemClient()
    ask_item_client.send_request("HandCamera_Value")

    while rclpy.ok():
        rclpy.spin_once(ask_item_client)
        if ask_item_client.future.done():
            try:
                response = ask_item_client.future.result()
                if response.ok:
                    ask_item_client.get_logger().info('OK')
                else:
                    ask_item_client.get_logger().info('not OK')
            except Exception as e:
                ask_item_client.get_logger().error(
                    'Service call failed %r' % (e,))
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
