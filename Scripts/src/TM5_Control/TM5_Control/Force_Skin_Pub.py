# import socket
# import select
# import time
# import threading
# import numpy as np
# import struct
# import rclpy
# import pygame
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
# from std_msgs.msg import Float64MultiArray

# buffer = []
# storage = []
# sublist = []
# Calibration_values = []
# dis = []
# global new_list
# new_list = []
# separated_lists = []
# global averages
# averages = []  # Initialize it
# touchDiffData = []
# size = 20
# counter = 0  # global counter
# bb = 1
# restarted = 0
# total = 0

# should_stop_readRaw = False
# should_stop_readCal = False
# channelDrive_global = None
# channelSensor_global = None
# should_stop_readRaw_print = True
# publisher = None
# ip = "10.0.0.47" # 185 / 58  / 47   / 109
# # ip = "192.168.10.5" # 185 / 58  / 47   / 109

# CELL_SIZE = 50
# GRID_OFFSET_X = 500
# GRID_OFFSET_Y = 50




# class Force_Data_Publisher(Node):
#     def __init__(self):
#         super().__init__('force_data_publisher')
#         self.publisher = self.create_publisher(Float64MultiArray, 'force_data_topic', 10)

#     def publish_force_data(self, data):
#         msg = Float64MultiArray()
#         msg.data = data
#         self.publisher.publish(msg)
#         self.get_logger().info(f'Publishing: {msg.data}')


# def send_command(command, ip, port):
#     global client_socket

#     client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#     try:
#         client_socket.connect((ip, port))
#         client_socket.setblocking(True)
#         print("Connected")

#         if command == "channelCheck":
#             client_socket.send(command.encode())
#             print("--------------------", flush=True)
#             print(f"Sending command: {command}", flush=True)
#             print("--------------------", flush=True)
#             channelCheck(client_socket)
#             client_socket.close()
#             print("Connection closed")


#         elif command == "readCal":
#             client_socket.send(command.encode())
#             print("--------------------", flush=True)
#             print(f"Sending command: {command}", flush=True)
#             print("--------------------", flush=True)
#             readCal(client_socket)
#             client_socket.close()
#             print("Connection closed")

#         elif command == "updateCal":
#             client_socket.send(command.encode())
#             print("--------------------", flush=True)
#             print(f"Sending command: {command}", flush=True)
#             print("--------------------", flush=True)
#             updateCal()
#             client_socket.close()
#             print("Connection closed")


#         elif command == "readRaw":
#             client_socket.send(command.encode())
#             print("--------------------", flush=True)
#             print(f"Sending command: {command}", flush=True)
#             print("--------------------", flush=True)
#             readRaw(client_socket)

#         # elif command == "getDistance":
#         #     print("--------------------", flush=True)
#         #     print(f"Sending command: {command}", flush=True)
#         #     print("--------------------", flush=True)
#         #     getDistance()

#         elif command == "oneKey":
#             print("--------------------", flush=True)
#             print(f"Sending command: {command}", flush=True)
#             print("--------------------", flush=True)
#             one_key()

#         elif command == "stop":
#             client_socket.send(command.encode())
#             print("--------------------", flush=True)
#             print(f"Sending command: {command}", flush=True)
#             print("--------------------", flush=True)
#             client_socket.close()
#             print("Connection closed")

#         else:
#             print("Unknown Command!!!", flush=True)

#     except Exception as e:
#         print(f"Error: {e}", flush=True)
#     finally:
#         print(f"cycle end: {command}", flush=True)
#         # client_socket.close()

# def channelCheck(client_socket):
#     global channelDrive_global, channelSensor_global

#     client_socket.setblocking(True)
#     response = client_socket.recv(12)

#     unpacked_data = struct.unpack('<6H', response)
#     print(unpacked_data, flush=True)
#     special_code_1, special_code_2, channelDrive, channelSensor, special_code_3, special_code_4 = unpacked_data
#     if special_code_1 == 55555 and special_code_2 == 55555 and special_code_3 == 44444 and special_code_4 == 44444:
#         print(f"Channels Ready: Sensor = {channelSensor}, Driver = {channelDrive}", flush=True)
#     else:
#         print("Data not formatted correctly or special codes mismatch", flush=True)

#     channelDrive_global = channelDrive
#     channelSensor_global = channelSensor

#     return channelDrive, channelSensor

# def readCal(client_socket):

#     global Calibration_values, no_of_elements, data_in_bytes, channelDrive_global, channelSensor_global

#     count = 1
#     no_of_elements = channelDrive_global * channelSensor_global

#     data_in_bytes = (no_of_elements + 4) * 2

#     while True:
#         if should_stop_readCal:
#             break
#         ready, _, _ = select.select([client_socket], [], [], 1.0)
#         if ready:
#             data = client_socket.recv(data_in_bytes)
#             hex_values = [data[i:i + 2] for i in range(0, len(data), 2)]
#             full_values = [int(hex_value[::-1].hex(), 16) for hex_value in hex_values]
#             Calibration_values = full_values[2:-2]
#             # print(f"readCal:", count,  Calibration_values, flush=True)
#             count += 1
#         if count == 6: # <-------- if you don't want to press the stop button after readCal, uncomment this two lines
#             print(f"readCal:", count, full_values, flush=True)
#             break      # <-------- if you don't want to press the stop button after readCal, uncomment this two lines

# def updateCal():
#     print("--------------------", flush=True)
#     print("Updated Calibration", flush=True)
#     print("--------------------", flush=True)

# def readRaw(client_socket):
#     global buffer, should_stop_readRaw, storage, sublist, list_storage, new_list, averages, separated_lists, should_stop_everything, restarted, total,Calibration_values
#     global force_data_publisher

#     one_time_flag = True
#     count = 1
#     should_stop_everything = False

#     while should_stop_everything is False:
#             if should_stop_readRaw:
#                 break
#             ready, _, _ = select.select([client_socket], [], [], 1.0)
#             if ready:
#                 data = client_socket.recv(data_in_bytes)
#                 hex_values = [data[i:i + 2] for i in range(0, len(data), 2)]
#                 decimal_values = [int(hex_value[::-1].hex(), 16) for hex_value in hex_values]
#                 buffer.extend(decimal_values)   # Save the data into buffer list

#                 if len(data) == 0 or len(data) is None:
#                     print("Received 'Please Reconnect' message from Arduino. Attempting to reconnect...")
#                     total = total + count
#                     restarted += 1
#                     send_command("readRaw", ip, 80)
#                 if one_time_flag:
#                     while len(buffer) >= 2 and (buffer[0] != 55555 or buffer[1] != 55555):
#                         buffer.pop(0)
#                     one_time_flag = len(buffer) < 2 or buffer[0] != 55555 or buffer[1] != 55555
#                 if len(buffer) < no_of_elements + 4:
#                     continue
#                 sublist = buffer[:no_of_elements + 4]
#                 if is_valid_sublist(sublist):
#                     storage.extend(sublist[2:no_of_elements + 2])
#                     buffer = buffer[no_of_elements + 4:]
#                     new_list.extend(sublist[2:no_of_elements + 2])
#                     if len(new_list) >= size * no_of_elements:
#                         for i in range(0, len(new_list), no_of_elements):
#                             separated_list = new_list[i:i + no_of_elements]
#                             separated_lists.append(separated_list)
#                         averages = []
#                         for elements in zip(*separated_lists):
#                             avg_value = sum(elements) / size
#                             averages.append(avg_value)
#                         new_list = []
#                         separated_lists = []
#                     matrix_readRaw = []
#                     for i in range(0, len(sublist[2:no_of_elements + 2]), channelSensor_global):
#                         row = sublist[2 + i: 2 + i + channelSensor_global]
#                         matrix_readRaw.append(row)
#                     if should_stop_readRaw_print is False:
#                         print(f"Package:", count, "Restarted:", restarted, "Total:", total, flush=True)
#                         matrix_readRaw = np.array(matrix_readRaw)-np.array(Calibration_values).reshape([11,11])
#                         # matrix_readRaw = np.array(matrix_readRaw).reshape([11,11])
#                         print(matrix_readRaw)
#                         # float_list = [float(item) for item in my_list]
#                         # matrix_readRaw = [float(mx) for mx in list(matrix_readRaw)]
#                         flat_matrix = [item for sublist in matrix_readRaw for item in sublist]
#                         float_list = [float(item) for item in flat_matrix]

#                         force_data_publisher.publish_force_data(float_list)
#                         # for row in matrix_readRaw:
#                         #     print(row, flush=True)
#                 else:
#                     index_first_55555 = find_first_55555_pair(sublist)
#                     if index_first_55555 is not None:
#                         buffer = buffer[index_first_55555:]
#                     else:
#                         print("Error occurs! Error occurs! Error occurs!!!", flush=True)
#                 count += 1

# def find_first_55555_pair(sublist):
#     return next((i for i in range(2, len(sublist) - 2) if sublist[i] == 55555 and sublist[i + 1] == 55555), None)

# def is_valid_sublist(sublist):
#     return sublist[:2] == [55555, 55555] and sublist[-2:] == [44444, 44444] and 55555 not in sublist[2:-2] and 44444 not in sublist[2:-2]


# def check_for_keypress():
#     print("There is nothing in this function right now ^^", flush=True)

# def one_key():
#     global should_stop_readRaw, should_stop_readRaw_print, should_stop_readCal, client_socket

#     print("--------------------", flush=True)
#     print('Entering one_key process..........')
#     print("--------------------", flush=True)

#     # time.sleep(3)
#     should_stop_readRaw = False
#     should_stop_readRaw_print = False

#     send_command("stop", ip,80)
#     time.sleep(1.5)

#     send_command("channelCheck", ip,80)
#     time.sleep(1.5)

#     send_command("readCal", ip, 80)
#     time.sleep(1.5)

#     send_command("updateCal", ip, 80)
#     time.sleep(1.5)

#     # threading.Thread(target=send_command, args=("readRaw", ip, 80)).start()
#     send_command("readRaw", ip, 80)
#     time.sleep(1.5)

#     # should_stop_readRaw_print = True
#     # threading.Thread(target=send_command, args=("getDistance", ip, 80)).start()
#     # time.sleep(1.5)

#     # threading.Thread(target=check_for_keypress).start()


# rclpy.init(args=None)
# force_data_publisher = Force_Data_Publisher()
# # one_key()
# def main():
#     one_key()
#     # publisher = node.create_publisher(Float32MultiArray, 'my_send_list', 10)
# #     rclpy.spin(node)

# if __name__ == '_main_':
#     main()

import sys
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import struct

class ArduinoROS2Bridge(Node):
    def __init__(self, serial_port, baud_rate):
        super().__init__('arduino_ros2_bridge')
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        try:
            self.ser = serial.Serial(serial_port, baud_rate)
            self.get_logger().info("Serial Connected")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port {serial_port} not found: {e}")
            sys.exit(1)

        self.publisher = self.create_publisher(Float64MultiArray, 'force_data_topic', 10)
        self.calibration_data = None
    def send_command(self, command):
        full_command = command + '\n'
        self.ser.write(full_command.encode())
        return self.read_response(command)

    def read_response(self, command, timeout=2):
        start_time = time.time()
        response_data = []
        while (time.time() - start_time) < timeout:
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode('utf-8').rstrip()
                # Process the response into a list
                if command in ["channelCheck", "readRaw", "updateCal", "readCal"]:
                    response_data = [int(value) for value in response.split() if value.isdigit()]
                    return response_data
        return response_data

    def channel_check(self):
        return self.send_command("channelCheck")

    def update_cal(self):
        return self.send_command("updateCal")

    def read_cal(self):
        return self.send_command("readCal")

    def read_raw(self):
        return self.send_command("readRaw")

    def publish_force_data(self, data):
        if self.calibration_data is None:
            # 第一次读到的数据用作校准数据
            self.calibration_data = data
        else:
            # 减去校准数据
            data = [current - base for current, base in zip(data, self.calibration_data)]
        # 转换数据为浮点数格式以便发布
        data = [float(item) for item in data]
        msg = Float64MultiArray()
        msg.data = data[2:-2]
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    serial_port = '/dev/ttyACM0'  # Update with your serial port
    baud_rate = 9600
    node = ArduinoROS2Bridge(serial_port, baud_rate)

    # Example usage
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            raw_data = node.read_raw()
            if raw_data:
                # Process and publish data
                node.publish_force_data(raw_data)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# ros2 run  TM5_Control Force_Skin_Pub 
# [INFO] [1710300007.581879117] [arduino_ros2_bridge]: Serial Connected
# Traceback (most recent call last):
#   File "/home/aaron/Humble_WS/install/TM5_Control/lib/TM5_Control/Force_Skin_Pub", line 33, in <module>
#     sys.exit(load_entry_point('TM5-Control==0.0.0', 'console_scripts', 'Force_Skin_Pub')())
#   File "/home/aaron/Humble_WS/install/TM5_Control/lib/python3.10/site-packages/TM5_Control/Force_Skin_Pub.py", line 376, in main
#     node.publish_force_data(raw_data)
#   File "/home/aaron/Humble_WS/install/TM5_Control/lib/python3.10/site-packages/TM5_Control/Force_Skin_Pub.py", line 358, in publish_force_data
#     msg.data = data
#   File "/opt/ros/humble/local/lib/python3.10/dist-packages/std_msgs/msg/_float64_multi_array.py", line 160, in data
#     assert \
# AssertionError: The 'data' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]
# [ros2run]: Process exited with failure 1
