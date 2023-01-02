import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose #importing required libraries

class fwd_kin(Node):

    def __init__(self):
        super().__init__('fwd_kin')
        self.publisher= self.create_publisher(				#fwd_kin for calculating forward kinematics
            Float64MultiArray,						#message type : Float32MultiArray
    	'fwd_kin',10)							#topic : frwd_kine_hw3
         								#callback function for frwd kinematics
       
        self.subscription = self.create_subscription(			#fwd_kin for calculating inverse kinematics
            JointState,							#message type : Pose
            'joint_states',						#topic : inv_kine_hw3
            self.callback_fwd,					#callback function for inv kinematics
            1)
        self.subscription  # this is to prevent unused variable warning

    def callback_fwd(self, msg):				#callback function for frwd kinematics
        
        q1=msg.data[0]							#taking the msg received from terminal
        q2=msg.data[1]
        q3=msg.data[2]
        
        print("")
        print("For Forward Kinematics:")
        l0 = 2.0								#assuming l0 link length as 2 units	
        l1 = 1.0								#assuming l1 link length as 1 units		
        l2 = 1.0								#assuming l2 link length as 1 units
        l3 = 1.0								#assuming l3 link length as 1 units
        									
        print("The link lengths are l1: ",l1, ", l2: ",l2,", l3: ",l3)
        P1 = np.array([[math.cos(q1),0, (-math.sin(q1)),0],[math.sin(q1),0,math.cos(q1),0],[0,-1,0,l1],[0,0,0,1]])				#matrix multiplication formula for P1*P2*P3 to get
        P2 = np.array([[math.cos(q2),(-math.sin(q2)),0,(l2*math.cos(q2))],[math.sin(q2),math.cos(q2),0,(l2*math.sin(q2))],[0,0,1,0],[0,0,0,1]])	#the final transformation matrix
        P3 = np.array([[math.cos(q3),(-math.sin(q3)),0,(l3*math.cos(q3))],[math.sin(q3),math.cos(q3),0,(l3*math.sin(q3))],[0,0,1,0],[0,0,0,1]])
                
        t = np.matmul(P1,P2)
        T = np.matmul(t, P3)
        print("The resultant Transformation Matrix T is :")
        print(T)
        
def main(args=None):
    rclpy.init(args=args)

    fwd_kin = fwd_kin()

    rclpy.spin(fwd_kin)						#used to keep the node running until an interrupt has been made


    fwd_kin.destroy_node()						#destroys the node an releases memory
    rclpy.shutdown()							#shuts down the node


if __name__ == '__main__':
    main()
