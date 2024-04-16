
import time
import numpy as np
import math 
import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import threading
from tm_msgs.msg import FeedbackState








"""
Control Publisher
"""
class PositionsPublisher(Node):
    def __init__(self):
        super().__init__('positions_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'positions_topic', 10)

    def publish_positions(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.publisher.publish(msg)
        # self.get_logger().info(f'Publishing: {msg.data}')


rclpy.init(args=None)
positions_publisher = PositionsPublisher()


def TM5_Thread():
    global alpha
    while 1:
        positions_publisher.publish_positions(list(alpha))
        time.sleep(0.3)

def main():
    i=0
    while 1:
        
        joint_pos = [417.50,-122.30,400.00,180.00,0.00,90.00]
        positions_publisher.publish_positions(list(joint_pos))
        time.sleep(1)
        joint_pos = [417.50,-122.30,300.00,180.00,0.00,90.00]
        positions_publisher.publish_positions(list(joint_pos))
        time.sleep(1)
if __name__ == '__main__':
    main()