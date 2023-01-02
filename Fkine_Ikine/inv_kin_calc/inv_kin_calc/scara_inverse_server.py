from invkin.srv import Invkin

import rclpy
from rclpy.node import Node
import math


class Service_ik(Node):

    def __init__(self):
        super().__init__('service_ik')
        self.srv = self.create_service(Invkin, 'inverse_kinematics', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        l0 = 1.0
        l1 = 1.0
        l2 = 1.0
        l3 = 1.0
        
        x = request.x
        y = request.y
        z = request.z
        
        d3 = l1 - z
        c2 = (x**2 + y**2 - l2**2 - l3**2)/(2*l2*l3)
        th2 = math.atan2((1-c2)**0.5,c2)
        th1 = math.atan2(y,x) - math.atan2(l3*math.sin(th2), l2 + l3*math.cos(th2))
        
        #write the formulae for inverse kinematics
        self.get_logger().info('Result of inverse_kinematics:\nx: %f y: %f z: %f' %(th1,th2,d3)
        return response

def main(args=None):
    rclpy.init(args=args)

    service_ik = Service_ik()

    rclpy.spin(service_ik)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
