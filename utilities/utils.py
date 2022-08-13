import queue
import threading
import time

import mavros_msgs.msg
import mavros_msgs.srv
import nav_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import math
import rospy
import numpy as np
from utilities.service import Service
import RPi.GPIO as GPIO

def nozzle_on():
	GPIO.output(33,GPIO.HIGH)

def nozzle_off():
	GPIO.output(33,GPIO.LOW)


def angle2radian(angle: float):
	if angle < 0:
		angle += 360.0
	return (angle * math.pi / 180)

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
	:param roll: The roll (rotation around x-axis) angle in radians.
	:param pitch: The pitch (rotation around y-axis) angle in radians.
	:param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
	:return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def quaternion_to_euler(w, x, y, z):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = np.degrees(np.arctan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = np.where(t2>+1.0,+1.0,t2)
	#t2 = +1.0 if t2 > +1.0 else t2

	t2 = np.where(t2<-1.0, -1.0, t2)
	#t2 = -1.0 if t2 < -1.0 else t2
	Y = np.degrees(np.arcsin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = np.degrees(np.arctan2(t3, t4))

	return X, Y, Z 

class Point:
	def __init__(self, x=0.0, y=0.0, z=0.0):
		self.x = x
		self.y = y
		self.z = z

	def add(self, wp):
		self.x += wp.x
		self.y += wp.y
		self.z += wp.z

	def mul(self, scale):
		self.x *= scale
		self.y *= scale
		self.z *= scale

class Waypoint(Point):
	def __init__(self, x=0.0, y=0.0, z=0.0, is_open=False):
		self.x = x
		self.y = y
		self.z = z
		self.is_open = is_open



MODE_MANUAL = "MANUAL"
MODE_ACRO = "ACRO"
MODE_LEARNING = "LEARNING"
MODE_STEERING = "STEERING"
MODE_HOLD = "HOLD"
MODE_LOITER = "LOITER"
MODE_FOLLOW = "FOLLOW"
MODE_SIMPLE = "SIMPLE"
MODE_AUTO = "AUTO"
MODE_RTL = "RTL"
MODE_SMART_RTL = "SMART_RTL"
MODE_GUIDED = "GUIDED"
MODE_INITIALISING = "INITIALISING"




class Config:
	def __init__(self):
		self.max_yaw_vel = 0.3
		self.font_scale = 1.0
		self.distance = 3.0

class Wall:
	def __init__(self):
		self.height = 4.0
		self.width = 8.0
		self.angle = 0.0
		self.is_init = False
		self.origin = Point()
		self.words = []
		self.satir_araligi = 1.0
		self.chars = {
		"A" : { "list" : [Waypoint(z=-3.0), Waypoint(x=1.0, z=3.0, is_open=True), Waypoint(x=1.0, z=-3.0, is_open=True), Waypoint(x=-1.5, z=1.5), Waypoint(x=1.0, is_open=True)],
			"width" : 2.25},
		"B" : { "list" : [Waypoint(z=-1.5), Waypoint(x=1.5, is_open=True), Waypoint(z=1.5, is_open=True), Waypoint(x=-1.5, is_open=True), Waypoint(z=-3.0, is_open=True), Waypoint(x=1.5, is_open=True), Waypoint(z=1.5, is_open=True)],
			"width" : 2.25},
		"C" : { "list" : [Waypoint(x=1.5), Waypoint(x=-1.5, is_open=True), Waypoint(z=-3.0, is_open=True), Waypoint(x=1.5, is_open=True)],
			"width" : 2.25},
		"D" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(x=1.5, is_open=True), Waypoint(z=3.0, is_open=True), Waypoint(-1.5, is_open=True)],
			"width" : 2.25},
		"G" : { "list" : [Waypoint(x=1.5), Waypoint(x=-1.5, is_open=True), Waypoint(z=-3.0, is_open=True), Waypoint(x=1.5, is_open=True), Waypoint(z=1.5, is_open=True), Waypoint(x=-0.75, is_open=True)],
			"width" : 2.25},
		"H" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(z=1.5), Waypoint(x=1.5, is_open=True), Waypoint(z=-1.5), Waypoint(z=3.0, is_open=True)],
			"width" : 2.25},
		"J" : { "list" : [Waypoint(z=-2.25), Waypoint(z=-0.75, is_open=True), Waypoint(x=1.5, is_open=True), Waypoint(z=3.0, is_open=True)],
			"width" : 2.25},
		"I" : { "list" : [Waypoint(z=-3.0, is_open=True)],
			"width" : 0.75},
		"İ" : { "list" : [Waypoint(z=-0.25, is_open=True), Waypoint(z=-0.25), Waypoint(z=-2.5, is_open=True)],
			"width" : 0.75},
		"L" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(x=1.5, is_open=True)],
			"width" : 2.25},
		"M" : { "list" : [Waypoint(z=-3.0), Waypoint(z=3.0, is_open=True), Waypoint(x=1.0, z=-1.5, is_open=True), Waypoint(x=1.0, z=1.5, is_open=True), Waypoint(z=-3.0, is_open=True)],
			"width" : 2.75},
		"P" : { "list" : [Waypoint(z=-1.5), Waypoint(x=1.5, is_open=True), Waypoint(z=1.5, is_open=True), Waypoint(x=-1.5, is_open=True), Waypoint(z=-3.0, is_open=True)],
			"width" : 2.25},
		"R" : { "list" : [Waypoint(z=-3.0), Waypoint(z=3.0, is_open=True), Waypoint(x=1.5, is_open=True), Waypoint(z=-1.5, is_open=True), Waypoint(x=-1.5, is_open=True), Waypoint(x=1.5, z=-1.5, is_open=True)],
			"width" : 2.25},
		"U" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(x=1.5, is_open=True), Waypoint(z=3.0, is_open=True)],
			"width" : 2.25},
		"V" : { "list" : [Waypoint(x=0.75, z=-3.0, is_open=True), Waypoint(x=0.75, z=3.0, is_open=True)],
			"width" : 2.25},
		"Y" : { "list" : [Waypoint(x=0.75, z=-1.5, is_open=True), Waypoint(z=-1.5, is_open=True), Waypoint(z=1.5), Waypoint(x=0.75, z=1.5, is_open=True)],
			"width" : 2.25},
		"Z" : { "list" : [Waypoint(x=1.5, is_open=True), Waypoint(x=-1.5, z=-3.0, is_open=True), Waypoint(x=1.5, is_open=True)],
			"width" : 2.25},
		"1" : { "list" : [Waypoint(z=-0.75), Waypoint(x=0.75, z=0.75, is_open=True), Waypoint(z=-3.0, is_open=True)],
			"width" : 1.5},
		"3" : { "list" : [Waypoint(x=1.5, is_open=True), Waypoint(z=-3.0, is_open=True), Waypoint(x=-1.5, is_open=True), Waypoint(z=1.5), Waypoint(x=1.5, is_open=True)],
			"width" : 2.25},
		"4" : { "list" : [Waypoint(z=-2.25), Waypoint(x=1.25, z=2.25, is_open=True), Waypoint(z=-3.0, is_open=True), Waypoint(x=-1.25, z=0.75), Waypoint(x=1.5, is_open=True)],
			"width" : 2.25},
		"5" : { "list" : [Waypoint(z=-3.0), Waypoint(x=1.5, is_open=True), Waypoint(z=1.5, is_open=True), Waypoint(x=-1.5, is_open=True), Waypoint(z=1.5, is_open=True), Waypoint(x=1.5, is_open=True)],
			"width" : 2.25},
		"6" : { "list" : [Waypoint(z=-1.5), Waypoint(x=1.5, is_open=True), Waypoint(z=-1.5, is_open=True), Waypoint(x=-1.5, is_open=True), Waypoint(z=3.0, is_open=True), Waypoint(x=1.5, is_open=True)],
			"width" : 2.25},
		"7" : { "list" : [Waypoint(x=1.5, is_open=True), Waypoint(x=0.75, z=-3.0, is_open=True)],
			"width" : 2.25},
		"8" : { "list" : [Waypoint(z=-1.5), Waypoint(x=1.5, is_open=True), Waypoint(z=-1.5, is_open=True), Waypoint(x=-1.5, is_open=True), Waypoint(z=3.0, is_open=True), Waypoint(x=1.5, is_open=True), Waypoint(z=-1.5, is_open=True)],
			"width" : 2.25},
		"9" : { "list" : [Waypoint(x=1.5, z=-3.0), Waypoint(z=3.0, is_open=True), Waypoint(x=-1.5, is_open=True), Waypoint(z=-1.5, is_open=True), Waypoint(x=1.5, is_open=True)],
			"width" : 2.25},
		"T" : { "list" : [Waypoint(x=2.0, is_open=True),Waypoint(x=-1.0),Waypoint(z=-3.0, is_open=True)],
			"width" : 2.75},
		"E" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(x=1.5, is_open=True), Waypoint(z=1.5), Waypoint(x=-1.5, is_open=True), Waypoint(z=1.5), Waypoint(x=1.5, is_open=True)],
			"width" : 2.25},
		"K" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(x=1.5), Waypoint(x=-1.5, z=1.5, is_open=True), Waypoint(x=1.5, z=1.5, is_open=True)],
			"width" : 2.25},
		"N" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(z=3.0), Waypoint(x=1.5 ,z=-3.0, is_open=True), Waypoint(z=3.0, is_open=True)],
			"width" : 2.25},
		"O" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(1.5, is_open=True), Waypoint(z=3.0, is_open=True), Waypoint(-1.5, is_open=True)],
			"width" : 2.25},
		"F" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(z=2.0), Waypoint(x=1.5, is_open=True), Waypoint(z=1.0), Waypoint(x=-1.5, is_open=True)],
			"width" : 2.25},
		"S" : { "list" : [Waypoint(z=-3.0), Waypoint(x=1.5, is_open=True), Waypoint(z=1.5, is_open=True), Waypoint(x=-1.5, is_open=True), Waypoint(z=1.5, is_open=True), Waypoint(x=1.5, is_open=True)],
			"width" : 2.25},
		"2" : { "list" : [Waypoint(1.5, is_open=True), Waypoint(z=-1.5, is_open=True), Waypoint(x=-1.5, is_open=True), Waypoint(z=-1.5, is_open=True), Waypoint(x=1.5, is_open=True)],
			"width" : 2.25},
		"0" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(1.5, is_open=True), Waypoint(z=3.0, is_open=True), Waypoint(-1.5, is_open=True)],
			"width" : 2.25},
		}

	
	def in_borders(self, x, y):
		if (x > 0.0) and (x < self.width):
			if (y > 0.0) and (y < self.height):
				return True
		return False

	def get_exact_loc(self, x, y, transform):
		(ret_x, ret_y) = transform(x, 0.0, angle=self.angle)
		ret_z = self.origin.z - y
		return ret_x, ret_y, ret_z



def get_distance(x1, y1, z1, x2, y2, z2):
	(dx, dy, dz) = (x1 - x2, y1 - y2, z1 - z2)
	return math.sqrt(math.pow(dx, 2)+math.pow(dy, 2)+math.pow(dz, 2))
