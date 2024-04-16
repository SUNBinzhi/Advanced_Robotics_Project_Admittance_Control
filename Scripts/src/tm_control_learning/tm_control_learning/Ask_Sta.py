import rclpy
from rclpy.node import Node
from tm_msgs.srv import AskSta
import sys

class AskStaClient(Node):
    def __init__(self):
        super().__init__('ask_sta_client')
        self.client = self.create_client(AskSta, 'ask_sta')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(1)
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, subcmd, subdata, wait_time):
        request = AskSta.Request()
        request.subcmd = subcmd
        request.subdata = subdata
        request.wait_time = wait_time
        self.future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    ask_sta_client = AskStaClient()
    ask_sta_client.send_request("00", "", 1)

    while rclpy.ok():
        rclpy.spin_once(ask_sta_client)
        if ask_sta_client.future.done():
            try:
                response = ask_sta_client.future.result()
                if response.ok:
                    ask_sta_client.get_logger().info('OK')
                    ask_sta_client.get_logger().info(response.subcmd)
                    ask_sta_client.get_logger().info(response.subdata)
                else:
                    ask_sta_client.get_logger().info('not OK')
            except Exception as e:
                ask_sta_client.get_logger().error(
                    'Service call failed %r' % (e,))
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
