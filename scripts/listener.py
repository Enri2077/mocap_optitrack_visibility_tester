#!/usr/bin/env python
import time
import pygame

import rospy
from std_msgs.msg import String
from turtlesim.msg import Pose

DETECT_TIME = 0.25	# time after which it is signaled that the pose has not changed

def beep():
	pygame.mixer.music.play()

class MyNode():
	def __init__(self):
		rospy.init_node('listener', anonymous=True)
		
		rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)
		
		pygame.init()
		pygame.mixer.music.load("audio/beep.wav")

		self.last_pose = None
		self.last_pose_t = None
		self.last_pose_change = None
		
		self.is_tracking_lost = False
 
	def pose_callback(self, pose):
		rospy.logdebug("received pose: x %s, y %s, z %s", pose.x, pose.y, 0)
	
		# if the pose is different from the last one received then the tracking isn't lost
		if not ( self.last_pose and pose.x == self.last_pose.x and pose.y == self.last_pose.y ):
			self.last_pose_change = time.time()		# update the last_pose_change's timestamp
			
			if self.is_tracking_lost:				# if it was lost before, log the new pose
				rospy.loginfo("Tracking jump to: x %s, y %s, z %s", pose.x, pose.y, 0)
			
			self.is_tracking_lost = False
		
		# if the pose hasn't changed for DETECT_TIME
		if self.last_pose_change and not self.is_tracking_lost and (time.time() - self.last_pose_change) > DETECT_TIME:
			self.is_tracking_lost = True			# the tracking is (probably) lost, log it
			rospy.loginfo("Tracking lost in: x %s, y %s, z %s", pose.x, pose.y, 0)
			beep()
		
		self.last_pose = pose
		self.last_pose_t = time.time()

if __name__ == '__main__':
	try:
		MyNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("ROSInterrupt")

