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
        if len(msg.joint_tor_max) == 6:
            self.get_logger().info(
                'the max torque is {}, {}, {}, {}, {}, {}'.format(
                    msg.joint_tor_max[0],
                    msg.joint_tor_max[1],
                    msg.joint_tor_max[2],
                    msg.joint_tor_max[3],
                    msg.joint_tor_max[4],
                    msg.joint_tor_max[5]))
        
        if len(msg.joint_tor_average) == 6:
            self.get_logger().info(
                'the average torque is {}, {}, {}, {}, {}, {}'.format(
                    msg.joint_tor_average[0],
                    msg.joint_tor_average[1],
                    msg.joint_tor_average[2],
                    msg.joint_tor_average[3],
                    msg.joint_tor_average[4],
                    msg.joint_tor_average[5]))

        if len(msg.joint_tor_min) == 6:
            self.get_logger().info(
                'the min torque is {}, {}, {}, {}, {}, {}'.format(
                    msg.joint_tor_min[0],
                    msg.joint_tor_min[1],
                    msg.joint_tor_min[2],
                    msg.joint_tor_min[3],
                    msg.joint_tor_min[4],
                    msg.joint_tor_min[5]))
            self.get_logger().info('---end this cycle---')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
