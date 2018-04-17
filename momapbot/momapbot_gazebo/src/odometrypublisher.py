#!/usr/bin/python

import tf
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import PoseWithCovariance
import numpy as np

class OdometryPublisher:
	def __init__(self):
		rospy.init_node('OdometryPublisher')
		#self.odometry_pub = rospy.Publisher(rospy.get_namespace()+'odom',Odometry,queue_size=1)
		self.odometry_pub = rospy.Publisher('/odom',Odometry,queue_size=1)
		self.model_state_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,self.publish_odometry)


	def publish_odometry(self,msg):
		self.msg = msg
		for i in xrange(0,len(msg.name)):
			if(msg.name[i]==rospy.get_namespace()[1:len(rospy.get_namespace())-1]):
				pose = PoseWithCovariance()
				twist = TwistWithCovariance()
				pose.pose = msg.pose[i]
				pose.pose.position.z=0

				twist.twist.angular.z = msg.twist[i].angular.z

				rot_matrix = tf.transformations.quaternion_matrix([msg.pose[i].orientation.x,msg.pose[i].orientation.y,msg.pose[i].orientation.z,msg.pose[i].orientation.w])
				lin_vel = np.matrix([msg.twist[i].linear.x,msg.twist[i].linear.y,msg.twist[i].linear.z])

				my_vel = rot_matrix[0:3,0:3].T*lin_vel.T

				xdot = my_vel.tolist()[0][0]

				twist.twist.linear.x = xdot

				# Fill the Odometry message
				odom_msg = Odometry()
				odom_msg.pose = pose
				odom_msg.twist = twist
				odom_msg.header.frame_id='/odom'
				odom_msg.child_frame_id=msg.name[i]
				odom_msg.header.stamp = rospy.Time.now()

				# Publish the Odometry message
				#self.odometry_pub.publish(odom_msg)


if __name__=='__main__':
	publisher = OdometryPublisher()
	rospy.spin()
