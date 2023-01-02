import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

import math
import numpy as np


class Robot_state_publisher(Node):

    def __init__(self):
        super().__init__('robot_state_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'Pose', 10)
        self.subscription = self.create_subscription(JointState,'/joint_states',self.listener_callback,1)
        self.subscription
        

    def listener_callback(self, msg):
    
        q1,q2,q3 = msg.position
        print(q1,q2,q3)
        l1=1 #assuming link length to be of 1 unit
        l2=1 #assuming link length to be of 1 unit
        l3=1 #assuming link length to be of 1 unit
        #l4=1 #assuming link length to be of 1 unit
        
        A1 = np.array([[math.cos(q1),(-math.sin(q1)),0,(l2*math.cos(q1))],[math.sin(q1),math.cos(q1),0,(l2*math.sin(q1))],[0,0,1,l1],[0,0,0,1]])
        A2 = np.array([[math.cos(q2),(math.sin(q2)),0,(l3*math.cos(q2))],[math.sin(q2),(-math.cos(q2)),0,(l3*math.sin(q2))],[0,0,-1,0],[0,0,0,1]])
        A3 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,q3],[0,0,0,1]])
        #print ("A1 = " ,A1)
        #print ("A2 = " ,A2)
        #print ("A3 = " ,A3)
        
        t = np.matmul(A1, A2)
        T = np.matmul(t, A3)
        
        x = T[0][3]
        y = T[1][3]
        z = T[2][3]
        
        pose_msg = Float64MultiArray()
        pose_msg.data = [x,y,z]
        
        self.publisher_.publish(pose_msg)

        print("The pose of the end-effector is:")
        print(T)


                     

def main(args=None):
    rclpy.init(args=args)
    robot_state_publisher = Robot_state_publisher()
    rclpy.spin(robot_state_publisher)


    robot_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
