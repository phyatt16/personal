#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
import tf
import numpy as np

class VelocityController:
	def __init__(self):
		rospy.init_node('VelocityController')
		self.cmd_vel_sub = rospy.Subscriber('cmd_vel',Twist,self.cmd_vel_cb)

		# THIS IS CHEATING - I GET GROUND TRUTH VELOCITY INSTEAD OF DIFFERENTIATING FROM TF
		self.vel_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,self.vel_cb)

		self.xdot = 0
		self.xddot = 0
		self.thetadot = 0
		self.thetaddot = 0
		self.xdot_cmd = 0
		self.thetadot_cmd = 0
		self.left_side_command = 0
		self.right_side_command = 0
		self.Kp_lin = 10
		self.Ki_lin = 0*.005
		self.Kp_ang = 10
		self.Ki_ang = 0*.005
		self.i_clamp = .1
		self.xdot_integrator_err = 0
		self.thetadot_integrator_err = 0
		self.rate = rospy.Rate(100)

		self.left_front_motor_pub = rospy.Publisher(rospy.get_namespace() + 'momapbot/left_front_wheel_velocity_controller/command',Float64,queue_size=1)
		self.left_rear_motor_pub = rospy.Publisher(rospy.get_namespace() + 'momapbot/left_rear_wheel_velocity_controller/command',Float64,queue_size=1)
		self.right_front_motor_pub = rospy.Publisher(rospy.get_namespace() + 'momapbot/right_front_wheel_velocity_controller/command',Float64,queue_size=1)
		self.right_rear_motor_pub = rospy.Publisher(rospy.get_namespace() + 'momapbot/right_rear_wheel_velocity_controller/command',Float64,queue_size=1)

	def vel_cb(self,msg):
		for i in xrange(0,len(msg.name)):
			if(msg.name[i]==rospy.get_namespace()[1:len(rospy.get_namespace())-1]):
				self.thetadot = msg.twist[i].angular.z

				rot_matrix = tf.transformations.quaternion_matrix([msg.pose[i].orientation.x,msg.pose[i].orientation.y,msg.pose[i].orientation.z,msg.pose[i].orientation.w])
				lin_vel = np.matrix([msg.twist[i].linear.x,msg.twist[i].linear.y,msg.twist[i].linear.z])

				my_vel = rot_matrix[0:3,0:3].T*lin_vel.T

				self.xdot = my_vel.tolist()[0][0]


	def cmd_vel_cb(self,msg):
		self.xdot_cmd = msg.linear.x
		self.thetadot_cmd = msg.angular.z

	def send_wheel_motor_commands(self):

		xdot_err = self.xdot_cmd - self.xdot
		self.xdot_integrator_err = self.xdot_integrator_err + xdot_err
		if(self.xdot_integrator_err>self.i_clamp):
			self.xdot_integrator_err=self.i_clamp
		elif(self.xdot_integrator_err<-self.i_clamp):
			self.xdot_integrator_err=-self.i_clamp

		thetadot_err = self.thetadot_cmd - self.thetadot
		self.thetadot_integrator_err = self.thetadot_integrator_err + thetadot_err
		if(self.thetadot_integrator_err>self.i_clamp):
			self.thetadot_integrator_err=self.i_clamp
		elif(self.thetadot_integrator_err<-self.i_clamp):
			self.thetadot_integrator_err=-self.i_clamp

		# Calculate command based on linear velocity error
		self.left_side_command = self.Kp_lin*xdot_err + self.Ki_lin*self.xdot_integrator_err
		self.right_side_command = self.Kp_lin*xdot_err + self.Ki_lin*self.xdot_integrator_err

		# Calculate command based on angular velocity error
		self.left_side_command = self.left_side_command - self.Kp_ang*thetadot_err - self.Ki_ang*self.thetadot_integrator_err
		self.right_side_command = self.right_side_command + self.Kp_ang*thetadot_err + self.Ki_ang*self.thetadot_integrator_err

		# Publish the commands to the wheel motors
		self.left_front_motor_pub.publish(self.left_side_command)
		self.left_rear_motor_pub.publish(self.left_side_command)
		self.right_front_motor_pub.publish(self.right_side_command)
		self.right_rear_motor_pub.publish(self.right_side_command)

	def control(self):
		while not rospy.is_shutdown():
			self.send_wheel_motor_commands()
			self.rate.sleep()
			'''
			print("xdot: ",self.xdot)
			print("xdot cmd: ",self.xdot_cmd)
			print("thetadot: ",self.thetadot)
			print("thetadot cmd: ",self.thetadot_cmd)
			'''

if __name__=='__main__':
	rospy.sleep(2)
	controller = VelocityController()
	controller.control()
