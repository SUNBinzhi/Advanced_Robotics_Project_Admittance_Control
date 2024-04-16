import rclpy
from rclpy.node import Node
from tm_msgs.msg import FeedbackState

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('demo_get_feedback')
        self.subscription = self.create_subscription(
            FeedbackState,
            'feedback_states',
            self.topic_callback,
            10)

    def topic_callback(self, msg):
        if len(msg.joint_pos) == 6:
            self.get_logger().info('FeedbackState: joint pos = ({}, {}, {}, {}, {}, {})'.format(
                msg.joint_pos[0],
                msg.joint_pos[1],
                msg.joint_pos[2],
                msg.joint_pos[3],
                msg.joint_pos[4],
                msg.joint_pos[5]))

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
