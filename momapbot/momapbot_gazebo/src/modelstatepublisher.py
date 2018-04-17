#!/usr/bin/python

import tf
import rospy
from gazebo_msgs.msg import ModelStates

class ModelStatePublisher:
	def __init__(self):
		rospy.init_node('ModelStatePublisher')
		self.model_state_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,self.publish_tf_transform)
		self.broadcaster = tf.TransformBroadcaster()

	def publish_tf_transform(self,msg):
		for i in xrange(0,len(msg.name)):
			#if(msg.name[i]==rospy.get_namespace()[1:len(rospy.get_namespace())-1]):
			if(msg.name[i]=='robot1'):


				self.broadcaster.sendTransform((msg.pose[i].position.x,msg.pose[i].position.y,msg.pose[i].position.z),
					(msg.pose[i].orientation.x,msg.pose[i].orientation.y,msg.pose[i].orientation.z,msg.pose[i].orientation.w),
					rospy.Time.now(),
					'base_link',
					#msg.name[i]+'/odom')
				    'odom')
				'''
				self.broadcaster.sendTransform((0,0,0),
					(0,0,0,1),
					rospy.Time.now()+rospy.Duration(.3),
					#msg.name[i]+'/odom',
					'odom',
					"/map")

				self.broadcaster.sendTransform((msg.pose[i].position.x,msg.pose[i].position.y,msg.pose[i].position.z),
					(msg.pose[i].orientation.x,msg.pose[i].orientation.y,msg.pose[i].orientation.z,msg.pose[i].orientation.w),
					rospy.Time.now(),
					msg.name[i]+'/base',
					"map")
	            '''

if __name__=='__main__':
	publisher = ModelStatePublisher()
	rospy.spin()
