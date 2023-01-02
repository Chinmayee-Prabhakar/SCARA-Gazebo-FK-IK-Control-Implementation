import sys

from invkin.srv import Invkin

import rclpy
from rclpy.node import Node

class client_ik(Node):

    def __init__(self):
        super().__init__('client_ik')
        self.cli = self.create_client(Invkin, 'inverse_kinematics')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Invkin.Request()

    def send_request(self, a, b, c):
        self.req.x = a
        self.req.y = b
        self.req.z = c
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = client_ik()
    response = minimal_client.send_request(float(sys.argv[1]),float(sys.argv[2]), float(sys.argv[3]))
    minimal_client.get_logger().info(
        'Sending: for %f %f %f' %
        (float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
