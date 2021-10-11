#! /usr/bin/python3

import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class Pursuer:
	def __init__(self):
		rospy.Subscriber('/turtle1/pose', Pose, self.callback1)
		rospy.Subscriber('/turtle2/pose', Pose, self.callback2)
		self.myY = 0
		self.myX = 0
		self.myTheta = 0
		self.cmdVel = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size = 1)

	def callback1(self, msg):
		newMsg = Twist()
		if self.checkDistance(msg):
			return
		newMsg.linear.x = math.sqrt((msg.x - self.myX) ** 2 + (msg.y - self.myY) ** 2) / 5
		newMsg.angular.z = math.atan2(msg.y - self.myY, msg.x - self.myX) - self.myTheta
		self.cmdVel.publish(newMsg)

	def callback2(self, msg):
		self.myX = msg.x
		self.myY = msg.y
		self.myTheta = msg.theta

	def checkDistance(self, msg):
		return abs(self.myX - msg.x) < 0.1 and abs(self.myY - msg.y) < 0.1


rospy.init_node('pursuer_turtle')
Pursuer()
rospy.spin()
