import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# class PositionsPublisher(Node):
#     def __init__(self):
#         super().__init__('positions_publisher')
#         self.publisher = self.create_publisher(Float64MultiArray, 'positions_topic', 10)
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)

#     def timer_callback(self):
#         msg = Float64MultiArray()
#         msg.data = [0.0, 0.0, 1.58, 0.0, 1.58, 0.0]  # 确保所有值都是浮点数
#         self.publisher.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.data)


# def main(args=None):
#     rclpy.init(args=args)

#     positions_publisher = PositionsPublisher()

#     rclpy.spin(positions_publisher)

#     positions_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLineEdit, QPushButton
from PyQt5.QtCore import pyqtSlot

from PyQt5.QtWidgets import QSlider, QLabel
from PyQt5.QtCore import Qt

from PyQt5.QtWidgets import QHBoxLayout, QLabel

class PositionsUI(QWidget):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher
        self.initUI()

    def initUI(self):
        self.layout = QVBoxLayout()
        self.sliders = []
        self.slider_values = []

        # 创建六个滑块及其标签
        for i in range(6):
            # 为每个滑块创建一个垂直布局
            slider_layout = QVBoxLayout()

            # 当前值标签
            value_label = QLabel('0.0', self)
            slider_layout.addWidget(value_label)
            self.slider_values.append(value_label)

            # 滑块
            slider = QSlider(Qt.Horizontal, self)
            slider.setMinimum(-50)  # 设置最小值
            slider.setMaximum(50)   # 设置最大值
            slider.setValue(0)       # 设置初始值
            slider.valueChanged[int].connect(lambda value, i=i: self.on_value_change(value, i))
            slider_layout.addWidget(slider)
            self.sliders.append(slider)

            # 将滑块布局添加到主布局
            self.layout.addLayout(slider_layout)

        # 发送按钮
        self.send_button = QPushButton('Send Positions', self)
        self.send_button.clicked.connect(self.on_click)
        self.layout.addWidget(self.send_button)

        self.setLayout(self.layout)
        self.setWindowTitle('Positions Publisher')
        self.show()

    def on_value_change(self, value, index):
        # 更新当前值标签
        self.slider_values[index].setText(f'{value / 100.0:.2f}')

    @pyqtSlot()
    def on_click(self):
        positions = [slider.value() / 100.0 for slider in self.sliders]
        self.publisher.publish_positions(positions)




def run_app(publisher):
    app = QApplication(sys.argv)
    ex = PositionsUI(publisher)
    sys.exit(app.exec_())


class PositionsPublisher(Node):
    def __init__(self):
        super().__init__('positions_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'positions_topic', 10)

    def publish_positions(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')




def main(args=None):
    rclpy.init(args=args)
    positions_publisher = PositionsPublisher()
    run_app(positions_publisher)  # 运行 UI
    # 如果 UI 关闭，则停止 ROS 2 节点
    positions_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#ros2 run tm_driver tm_driver robot_ip:=10.0.0.43
#ros2 run tm_driver tm_driver robot_ip:=10.0.0.53