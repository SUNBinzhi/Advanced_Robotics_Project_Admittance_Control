# ModbusTCP -- TM5&FT300-s
# Retrieve data -- Demo v1.0

# from pyModbusTCP.client import ModbusClient as ModbusClientTCP
# from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import numpy as np
import time
import threading


def reinterpredSigned16(val_array):
    for idx, val in enumerate(val_array):
        if val >= 32768:
            val_array[idx] = val - 65536


class ModbusRobotIQ:
    def __init__(self, method="rtu", port="ttyUSB0", stopbits=1, bytesize=8, parity='N', baudrate=19200):
        self.client = ModbusClient(method=method, port=port, stopbits=stopbits,
                                   bytesize=bytesize, parity=parity, baudrate=baudrate)

        # print(connection)
        self.lock = threading.Lock()

    def connect(self):
        connection = self.client.connect()
        print(f'[ft sensor] connection: {connection}')

    def disconnect(self):
        self.client.close()
        print(f'[ft sensor] disconnect')

    def get_data(self):

        results = np.zeros([1, 6]).reshape([1, 6])
        # print(results.shape)

        try:
            self.lock.acquire()
            # print(self.client)
            request = self.client.read_holding_registers(address=180, count=6, slave=0, unit=9)
            # print(request)
            raw_results = np.array(request.registers)
            reinterpredSigned16(raw_results)
            results = np.array(raw_results).reshape([1, 6])
            # print(results.shape)# .flatten().tolist()
            # print(type(results))
            self.lock.release()
        except:
            print(f'[modbus_robotiq]: got empty position array')
            pass

        return results


if __name__ == '__main__':

    # force-torque sensor
    client_ft = ModbusRobotIQ(method="rtu", port="ttyACM0", stopbits=1, bytesize=8, parity='N', baudrate=19200)

    while 1:
        time.sleep(1)
        res = client_ft.get_data()
        print(res)
