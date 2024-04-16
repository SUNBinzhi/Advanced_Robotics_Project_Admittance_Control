import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import time
import mujoco as mj


class PositionsPublisher(Node):
    def __init__(self):
        super().__init__('positions_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'positions_topic', 10)

    def publish_positions(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')




# def main(args=None):
#     rclpy.init(args=args)
#     positions_publisher = PositionsPublisher()
#     run_app(positions_publisher)  # 运行 UI
#     # 如果 UI 关闭，则停止 ROS 2 节点
#     positions_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#ros2 run tm_driver tm_driver robot_ip:=10.0.0.43
#ros2 run tm_driver tm_driver robot_ip:=10.0.0.53