#!/usr/bin/env python
import time
import pygame

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point


DETECT_TIME = 0.25	# time after which it is signaled that the pose has not changed

def beep():
	pygame.mixer.music.play()

class MyNode():
	def __init__(self):
		rospy.init_node('listener', anonymous=True)
		
		rospy.Subscriber("/Robot_1/pose", PoseStamped, self.pose_callback)
		
		pygame_init = pygame.init()
		
		rospy.loginfo(pygame_init)
		
		pygame.mixer.music.load("/home/enrico/workspace-ros/src/mocap_optitrack_visibility_tester/audio/beep.wav")

		self.last_pose = None
		self.last_pose_t = None
		self.last_pose_change = None
		
		self.is_tracking_lost = False
 
	def pose_callback(self, pose):
		rospy.logdebug("received pose: x %s, y %s, z %s", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
		# if the pose is different from the last one received then the tracking isn't lost
		if not ( self.last_pose and pose.pose.position.x == self.last_pose.pose.position.x and pose.pose.position.y == self.last_pose.pose.position.y and pose.pose.position.z == self.last_pose.pose.position.z ):
			self.last_pose_change = time.time()		# update the last_pose_change's timestamp
			
			if self.is_tracking_lost:				# if it was lost before, log the new pose
				rospy.loginfo("Tracking jump to: x %s, y %s, z %s", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
			
			self.is_tracking_lost = False
		
		# if the pose hasn't changed for DETECT_TIME
		if self.last_pose_change and not self.is_tracking_lost and (time.time() - self.last_pose_change) > DETECT_TIME:
			self.is_tracking_lost = True			# the tracking is (probably) lost, log it
			rospy.loginfo("Tracking lost in: x %s, y %s, z %s", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
			beep()
		
		self.last_pose = pose
		self.last_pose_t = time.time()

if __name__ == '__main__':
	try:
		MyNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("ROSInterrupt")

