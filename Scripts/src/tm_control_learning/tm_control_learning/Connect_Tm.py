# demo_connect_tm:
# In this demo code, the user can set the connection type.
# If the user sets reconnect to true, every time the driver disconnects from the Listen node,
#  it will try to reconnect.
# There are two kinds of connection settings the user can select, 
# one is "connect_tmsvr" for Ethernet server connection, and the other is "connect_tmsct" for TMflow connection.




import rclpy
from rclpy.node import Node
from tm_msgs.srv import ConnectTM
import sys

class ConnectTMClient(Node):
    def __init__(self):
        super().__init__('connect_tm_client')
        self.client = self.create_client(ConnectTM, 'connect_tmsvr')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(1)
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self):
        request = ConnectTM.Request()
        request.server = ConnectTM.Request.TMSVR
        request.reconnect = True
        request.timeout = 0
        request.timeval = 0
        self.future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    connect_tm_client = ConnectTMClient()
    connect_tm_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(connect_tm_client)
        if connect_tm_client.future.done():
            try:
                response = connect_tm_client.future.result()
                if response.ok:
                    connect_tm_client.get_logger().info('OK')
                else:
                    connect_tm_client.get_logger().info('not OK')
            except Exception as e:
                connect_tm_client.get_logger().error(
                    'Service call failed %r' % (e,))
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
