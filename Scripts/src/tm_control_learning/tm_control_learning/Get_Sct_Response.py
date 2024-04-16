import rclpy
from rclpy.node import Node
from tm_msgs.msg import SctResponse

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('demo_get_sct_response')
        self.subscription = self.create_subscription(
            SctResponse,
            'sct_response',
            self.topic_callback,
            10)

    def topic_callback(self, msg):
        self.get_logger().info('SctResponse: id is = {}, script is {}'.format(
            msg.id, msg.script))

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
