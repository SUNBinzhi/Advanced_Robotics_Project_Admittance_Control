import rclpy
from rclpy.node import Node
from tm_msgs.msg import StaResponse

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('demo_get_sta_response')
        self.subscription = self.create_subscription(
            StaResponse,
            'sta_response',
            self.topic_callback,
            10)

    def topic_callback(self, msg):
        self.get_logger().info('StaResponse: subcmd is = {}, subdata is {}'.format(
            msg.subcmd, msg.subdata))

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
