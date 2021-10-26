#! /usr/bin/python

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

# filter settings
FILTER_STEP = 3
FILTER_MAX_VALUE = 1000.0
FILTER_MAX_DISTANCE = 0.5


# pub = rospy.Publisher('/lazer_scan', LaserScan, queue_size = 10)
filterPub = rospy.Publisher('/filtered_lazer_scan', LaserScan, queue_size = 10)

def polar2cart(r, angle):
    return r * math.cos(angle), r * math.sin(angle)

def getAngle(min, step, pos):
    return min + step * pos

def checkDistance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def clearRanges(ranges):
	for i in range(FILTER_STEP):
		ranges.append(FILTER_MAX_VALUE)
	return ranges

def filterRange(msg):
	filteredRanges = []
	for i in range(FILTER_STEP):
		filteredRanges.append(FILTER_MAX_VALUE)

	for i in range(FILTER_STEP, len(msg.ranges) - FILTER_STEP):
		leftPoint  = polar2cart(msg.ranges[i - FILTER_STEP], getAngle(msg.angle_min, msg.angle_increment, i - FILTER_STEP))
		point 	   = polar2cart(msg.ranges[i], getAngle(msg.angle_min, msg.angle_increment, i))
		rightPoint = polar2cart(msg.ranges[i + FILTER_STEP], getAngle(msg.angle_min, msg.angle_increment, i + FILTER_STEP))

		if checkDistance(leftPoint, point) > FILTER_MAX_DISTANCE or checkDistance(rightPoint, point) > FILTER_MAX_DISTANCE:
			filteredRanges.append(FILTER_MAX_VALUE)
		else:
			filteredRanges.append(msg.ranges[i])

	for i in range(FILTER_STEP):
		filteredRanges.append(FILTER_MAX_VALUE)
	return filteredRanges

def callback(msg):
	filteredRanges = filterRange(msg)
	# print(filteredRanges)
	msg.ranges = filteredRanges
	filterPub.publish(msg)

rospy.init_node('scan_data')
sub = rospy.Subscriber('/base_scan', LaserScan, callback)
# print(ranges)
rospy.spin()
