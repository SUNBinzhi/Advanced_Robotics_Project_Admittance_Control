import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'positions_topic', 10)
        time.sleep(1)  # 留出时间确保ROS 2网络准备就绪

    def publish_positions_with_duration(self, positions, duration,velocity):
        msg = Float64MultiArray()
        msg.data = positions + [duration]+velocity  # 将位置和持续时间合并为一个列表
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    publisher_node = PositionPublisher()

    try:
        z_value = 300.0
        while rclpy.ok() and z_value <=500.0:
            # 设置期望的TCP位置，这里改变Z值，并且添加持续时间
            positions = [417.50, -122.30, z_value, 180.00, 0.00, 90.00]
            velocity = [0.0,0.0,0.0,0.0,0.0,0.0]
            duration = 0.1  # 设置持续时间为0.1秒
            publisher_node.publish_positions_with_duration(positions, duration,velocity)
            
            # 等待0.1秒以模拟循环间隔
            time.sleep(0.1)
            
            # 更新Z值，这里每次循环增加1单位
            z_value=z_value+1


        # z_value = 500.0
        while rclpy.ok() and z_value >=300.0:
            # 设置期望的TCP位置，这里改变Z值，并且添加持续时间
            positions = [417.50, -122.30, z_value, 180.00, 0.00, 90.00]
            velocity = [0.0,0.0,0.0,0.0,0.0,0.0]

            duration = 0.1# 设置持续时间为0.1秒
            publisher_node.publish_positions_with_duration(positions, duration,velocity)
            
            # 等待0.1秒以模拟循环间隔
            time.sleep(0.1)
            
            # 更新Z值，这里每次循环增加1单位
            z_value=z_value-1

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
