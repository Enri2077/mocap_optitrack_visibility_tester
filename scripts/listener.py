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
		
		pygame.mixer.music.load("audio/beep.wav")
		
		beep()
		
		rospy.loginfo("sound initialized: trial beep should be heard")

		self.last_pose = None
		self.last_pose_t = None
		self.last_pose_change = None
		
		self.is_tracking_lost = False
 
	def pose_callback(self, pose_stamped):
		pos = pose_stamped.pose.position
		if(self.last_pose):	last_pos = self.last_pose.pose.position
		else:	last_pos = None
		
		rospy.logdebug("received pose: x %s, y %s, z %s", pos.x, pos.y, pos.z)
		# if the pose is different from the last one received then the tracking isn't lost
		if not ( self.last_pose and pos.x == last_pos.x and pos.y == last_pos.y and pos.z == last_pos.z ):
			self.last_pose_change = time.time()		# update the last_pose_change's timestamp
			
			if self.is_tracking_lost:				# if it was lost before, log the new pose
				rospy.loginfo("Tracking jump to: x %s, y %s, z %s", pos.x, pos.y, pos.z)
			
			self.is_tracking_lost = False
		
		# if the pose hasn't changed for DETECT_TIME
		if self.last_pose_change and not self.is_tracking_lost and (time.time() - self.last_pose_change) > DETECT_TIME:
			self.is_tracking_lost = True			# the tracking is (probably) lost, log it
			rospy.loginfo("Tracking lost in: x %s, y %s, z %s", pos.x, pos.y, pos.z)
			beep()
		
		self.last_pose = pose_stamped
		self.last_pose_t = time.time()

if __name__ == '__main__':
	try:
		MyNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("ROSInterrupt")

