import rclpy
import numpy as np
import math
from rclpy.node import Node
from srvs.srv import EEV, JV
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class Pid_controller(Node):

    def __init__(self):
        super().__init__('pid_controller')
        self.joint_vel = Float64MultiArray() 
        self.joint_vel.data = [0.0,0.0,0.0] 
        self.pos = [0,0,0] #global position array
        self.vel = [0,0,0] #global effort array
        self.err = [0,0,0] #global error array
        self.vs = [0,0,0] #global q1,q2 and d3
        self.kp = [0.5,0.5,1] #defining the kp for all the 3 joints
        self.kd = [0.01,0.01,1] #defining the kd for all the 3 joints
        self.i = 0
        self.q = [0.0, 0.0, 0.0]
        self.q_dot = [0.0, 0.0, 0.0]
        self.prev_e = [0.0, 0.0, 0.0]
        self.u = [0.0, 0.0, 0.0]
        self.v_end = [0.0, 0.0, 0.0]
        self.q_ref = [0.0, 0.0, 0.0]
        self.q_dot_ref = [0.0, 0.0, 0.0]
        self.ref_vel = [0.0, 0.0, 0.0] #v_end_ref
	
	#JOINT STATES SUBSCRIBER
        self.subscription = self.create_subscription(JointState,'/joint_states',self.current_pos_callback,10)
        self.subscription

        self.publisher_ = self.create_publisher(Float64MultiArray,'/forward_effort_controller/commands',10)

        self.srv1 = self.create_service(JV,'jv',self.jv_callback_srv)
        self.srv2 = self.create_service(EEV,'ev',self.eev_callback_srv)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.listener_callback)
        #open file and creater writer
	self.f = open('/home/anuj/Desktop/plot.csv','w')
	self.writer = csv.writer(self.f)
	self.writer.writerow(['q1_dot','q2_dot','q3_dot','u1','u2'])

    def jv_callback_srv(self, request, response):
        l = [1.0,1.0,1.0]
        c1 = math.cos(self.q1)
        c2 = math.cos(self.q2)
        s1 = math.sin(self.q1)
        s2 = math.sin(self.q2)
        J = np.array([[((-s1*c2) + s1 + (c1*s2)),((-s1*c2)-(c1*s2)),0],[((c1*s2)-c1-(s1*s2)),((c1*s2)-(s1*s2)),0],[0,0,-1],[0,0,0],[0,0,0],[1,1,0]],dtype=float)
        q_dot_arr = np.array([[request.v1], [request.v2], [request.v3]],dtype=float)
        q_dot = np.matmul(J, q_dot_arr)
        response.eev1 = q_dot[0,0]
        response.eev2 = q_dot[1,0]
        response.eev3 = q_dot[2,0]
        return response
    
    def eev_callback_srv(self, request, response):
        l = [1.0,1.0,1.0]
        J = np.array([[((-s1*c2) + s1 + (c1*s2)),((-s1*c2)-(c1*s2)),0],[((c1*s2)-c1-(s1*s2)),((c1*s2)-(s1*s2)),0],[0,0,-1],[0,0,0],[0,0,0],[1,1,0]],dtype=float)
        q_dot_arr = np.array([[request.ev1], [request.ev2], [request.ev3]],dtype=float)
        q_dot = np.matmul(np.linalg.pinv(J)[:,0:3], q_dot_arr)
        response.v1 = q_dot[0,0]
        response.v2 = q_dot[1,0]
        response.v3 = q_dot[2,0]
        self.q1_dot_ref = q_dot[0,0]
        self.q2_dot_ref = q_dot[1,0]
        self.q3_dot_ref = q_dot[2,0]
        return response
    
    def current_pos_callback(self, msg):
        #self.pos = msg.position #current joint position value array
        self.vel = msg.velocity #current joint velocity value array
        #self.effort = msg.effort #current joint effort value array
        self.ref_vel = [0.0, 1, 0.0] #EE REFERENCE VELOCITY
        c1 = math.cos(self.q1)
        c2 = math.cos(self.q2)
        s1 = math.sin(self.q1)
        s2 = math.sin(self.q2)
        J = np.array([[((-s1*c2) + s1 + (c1*s2)),((-s1*c2)-(c1*s2)),0],[((c1*s2)-c1-(s1*s2)),((c1*s2)-(s1*s2)),0],[0,0,-1],[0,0,0],[0,0,0],[1,1,0]],dtype=float)
        q_dot = np.matmul(np.linalg.pinv(J)[:,0:3], np.array(ref_vel))
        self.q1_dot_ref = q_dot[0,0]
        self.q2_dot_ref = q_dot[1,0]
        self.q3_dot_ref = q_dot[2,0]
    def close_file(self):
		self.f.close()

    def listener_callback(self):
        sec = msg.header.stamp.sec
        ms_1 = 0
        nanosec = msg.header.stamp.nanosec
        ms = ((sec/1000)+(nanosec*100000))
        if (self.err)!=0:
        	for i in range(3): 
        		self.err[i] = self.ref_vel[i] - self.vel[i] #calculating error 
        		self.e_err[i] += (self.err[i])/(ms-ms_1)
        		self.d_err[i] = (self.err[i]-last_err[i])/(ms-ms_1)
        		last_err[i] = self.err[i]
        		self.joint_vel.data[i] = (self.kp[i]* self.vel[i] + self.d_err[i]*self.kd[i]) #updating joint effort using PD controller
        else:
        	self.joint_vel.data = [0.0, 0.0, 0.0] #reset the joint effort
        	for i in range(len(self.kp)):
        		self.err[i] = self.vel[i] - self.ref_vel[i]

        self.publisher_.publish(self.joint_effort)
        
        	
        self.i+=1
def main(args=None):
    rclpy.init(args=args)
    #initialize the node
    pid_controller = Pid_controller()
    
    #keep it running till keyboard interrupt occurs
    rclpy.spin(pid_controller)
    pid_controller.close_file()
    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
