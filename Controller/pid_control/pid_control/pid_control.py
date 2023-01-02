import rclpy
from rclpy.node import Node
from invkin.srv import PID
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class Pid_controller(Node):

    def __init__(self):
        super().__init__('pid_controller')
        self.joint_effort = Float64MultiArray() 
        self.joint_effort.data = [0.0,0.0,0.0] 
        self.pos = [0,0,0] #global position array
        self.err = [0,0,0] #global error array
        self.qds = [0,0,0] #global q1,q2 and d3
        self.kp = [16,14,25] #defining the kp for all the 3 joints
        self.kd = [13,13,18] #defining the kd for all the 3 joints
        self.i = 0
        self.subscription = self.create_subscription(JointState,'/joint_states',self.current_pos_callback,10)
        self.subscription
        self.publisher_ = self.create_publisher(Float64MultiArray,'/forward_effort_controller/commands',10)
        self.srv = self.create_service(PID, 'inverse_kinematics', self.effort_callback)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.listener_callback)
        
    def effort_callback(self, request, response):
        self.qds = [request.x, request.y, request.z] 
        response.msg = "Reference Values fetched"
        return response
        
    def current_pos_callback(self, msg):
    	self.pos = msg.position #current joint position value array
    	self.vel = msg.velocity #current joint velocity value array
    	
    def listener_callback(self):
        
        #dt = 0.01
        
        if (self.err)!=0:
        
        	for i in range(len(self.kp)): 
        		self.err[i] = self.pos[i] - self.qds[i] #calculating error 
        		self.joint_effort.data[i] = -(self.kp[i]* self.err[i] + self.vel[i]*self.kd[i]) #updating joint effort using PD controller
        else:
        	self.joint_effort.data = [0.0, 0.0, 0.0] #reset the joint effort
        	for i in range(len(self.kp)):
        		self.err[i] = self.pos[i] - self.qds[i]

        self.publisher_.publish(self.joint_effort)
        if self.i<=1000: 	#write the data to a file
        	f = open('J1.txt', 'a')
        	f.write(str(self.pos[0]))
        	f.write("\n")
        	f.close()
        
        	f = open('J2.txt', 'a')
        	f.write(str(self.pos[1]))
        	f.write("\n")
        	f.close()
        	
        	f = open('J3.txt', 'a')
        	f.write(str(self.pos[2]))
        	f.write("\n")
        	f.close()
        	
        self.i+=1
def main(args=None):
    rclpy.init(args=args)
    #initialize the node
    pid_controller = Pid_controller()
    
    #keep it running till keyboard interrupt occurs
    rclpy.spin(pid_controller)
    
    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
