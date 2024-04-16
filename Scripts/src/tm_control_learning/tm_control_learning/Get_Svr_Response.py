import rclpy
from rclpy.node import Node
from tm_msgs.msg import SvrResponse

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('demo_get_svr_response')
        self.subscription = self.create_subscription(
            SvrResponse,
            'svr_response',
            self.topic_callback,
            10)

    def topic_callback(self, msg):
        self.get_logger().info(
            'SvrResponse: id is = {}, mode is {}, content is {}, error code is {}'.format(
                msg.id, int(msg.mode), msg.content, int(msg.error_code)))

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
