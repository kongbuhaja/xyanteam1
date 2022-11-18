#!/usr/bin/env python3

import rospy
import time
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import LaserScan
import math
import numpy

class car():
	def __init__(self):
		self.speed=3
		self.angle=0
		self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_sub_callback, queue_size=1)
		self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
		self.motor_control = xycar_motor()
		self.motor_control.speed=3
		self.motor_control.angle=0
		self.ini = dict()
		self.lidar_points=None
	def lidar_sub_callback(self, data):
		self.lidar_points=data.ranges
	def is_lidar_on(self):
		return True if self.lidar_points==None else False
	def motor_pub(self):
		self.motor_pub.publish(self.motor_control)
	def set_motor_control(self):
		self.motor_control.angle = self.angle
		self.motor_control.speed = self.speed
	def stop(self, threshold = 0.30):
		if self.lidar_points == None:
			return
		start=350
		end=110
		front = self.lidar_points[start:] + self.lidar_points[:end] 
		count=20
		for i, f in enumerate(front):
			if f <= threshold:
				count-=1
				if count<=0:
					print((start+i)%505)
					self.speed*=-1
					return
			else:
				count=20

	def run(self):
		self.stop()
		self.set_motor_control()
		self.motor_pub.publish(self.motor_control)

rospy.init_node('auto_driver')
xycar = car()

time.sleep(3)
while not rospy.is_shutdown():
	xycar.run()
	
