import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class SensorDataSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_data_subscriber')
        self.calibation_data = None
        self.data = None
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'force_data_topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # if self.calibation_data is None:
        #     self.calibration_data = msg.data
        # else:
        #     self.data = [value-base for value,base in zip(msg.date,self.calibration_data)]
        self.data = msg.data


import sys
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QLabel
from PyQt5.QtCore import Qt, QTimer

class SensorDataWidget(QWidget):
    def __init__(self, sensor_data_subscriber):
        super().__init__()
        self.subscriber = sensor_data_subscriber
        self.initUI()

    def initUI(self):
        self.grid = QGridLayout()
        self.labels = [QLabel('0') for _ in range(121)]  # 11x11 grid
        for i, label in enumerate(self.labels):
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumSize(60, 60)
            self.grid.addWidget(label, i // 11, i % 11)
        self.setLayout(self.grid)
        self.resize(800, 800)
        self.updateData()

    def updateData(self):
        if self.subscriber.data:
            for i, value in enumerate(self.subscriber.data):
                negated_value = -value  # Negate each value individually
                color = self.calculateColor(negated_value)
                self.labels[i].setStyleSheet(f"background-color: {color};")
                self.labels[i].setText(str(negated_value))
        self.update()  # Update UI
        QTimer.singleShot(10, self.updateData)  # Refresh every 100 milliseconds


    # def calculateColor(self, value):
    #     intensity = int((value / 10000) * 255)
    #     return f'rgb({intensity}, {255 - intensity}, 0)'
    # def calculateColor(self, value):
        
    #     normalized_value = (value + 10000) / 2  # 将范围从 -10000-10000 规范化到 0-10000
    #     intensity = int((normalized_value / 10000) * 255)
    #     return f'rgb({intensity}, {255 - intensity}, 0)'
    def calculateColor(self, value):
        # Assuming sensor values range from 0 to 30000 for visualization
        min_val, max_val = -3000, 3000
        # Normalize the value within the range [0, 255] for RGB color mapping
        normalized_value = max(min((value - min_val) / (max_val - min_val), 1), 0)
        intensity = int(normalized_value * 255)
        # Create a gradient from blue (low) to red (high)
        return f'rgb({intensity}, {128 - int(normalized_value * 128)}, {255 - intensity})'

def main():
    rclpy.init(args=None)
    sensor_data_subscriber = SensorDataSubscriber()

    app = QApplication(sys.argv)
    ex = SensorDataWidget(sensor_data_subscriber)
    ex.show()

    while rclpy.ok():
        rclpy.spin_once(sensor_data_subscriber)
        app.processEvents()

    sensor_data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
