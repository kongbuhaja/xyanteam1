#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan

class lidar():
	lidar_points=None
	def lidar_callback(self, data):
		self.lidar_points = data.ranges
	def is_none(self):
		return True if self.lidar_points==None else False
lidar_=lidar()
rospy.init_node("my_lidar", anonymous=True)
rospy.Subscriber("/scan", LaserScan, lidar_.lidar_callback, queue_size=1)

while not rospy.is_shutdown():
	if lidar_.is_none():
		continue
	rtn=""
	for i in range(12):
		rtn += str(format(lidar_.lidar_points[i*30], '.2f')) +", "
	print(rtn[:-2])
	time.sleep(1.0)
