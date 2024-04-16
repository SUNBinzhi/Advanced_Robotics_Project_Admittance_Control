import rclpy
from rclpy.node import Node
from tm_msgs.srv import WriteItem
import sys

class WriteItemClient(Node):
    def __init__(self):
        super().__init__('write_item_client')
        self.client = self.create_client(WriteItem, 'write_item')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(1)
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, id, item, value):
        request = WriteItem.Request()
        request.id = id
        request.item = item
        request.value = value
        self.future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    write_item_client = WriteItemClient()
    write_item_client.send_request("123", "Ctrl_DO0", "1")

    while rclpy.ok():
        rclpy.spin_once(write_item_client)
        if write_item_client.future.done():
            try:
                response = write_item_client.future.result()
                if response.ok:
                    write_item_client.get_logger().info('OK')
                else:
                    write_item_client.get_logger().info('not OK')
            except Exception as e:
                write_item_client.get_logger().error(
                    'Service call failed %r' % (e,))
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
