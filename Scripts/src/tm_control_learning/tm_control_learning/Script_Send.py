import rclpy
from rclpy.node import Node
from tm_msgs.srv import SendScript
import sys

class SendScriptClient(Node):
    def __init__(self):
        super().__init__('send_script_client')
        self.client = self.create_client(SendScript, 'send_script')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(1)
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, cmd):
        request = SendScript.Request()
        request.id = 'demo'
        request.script = cmd
        self.future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    send_script_client = SendScriptClient()
    #(Point-To-Point) 
    cmd = "PTP(\"JPP\",0,0,0,0,0,0,70,200,0,true)"
    send_script_client.send_request(cmd)

    while rclpy.ok():
        rclpy.spin_once(send_script_client)
        if send_script_client.future.done():
            try:
                response = send_script_client.future.result()
                if response.ok:
                    send_script_client.get_logger().info('OK')
                else:
                    send_script_client.get_logger().info('not OK')
            except Exception as e:
                send_script_client.get_logger().error(
                    'Service call failed %r' % (e,))
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
