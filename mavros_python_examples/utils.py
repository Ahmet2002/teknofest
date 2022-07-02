import numpy as np
import math

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

class Waypoint:
    def __init__(self, x=0.0, y=0.0, z=0.0, is_open=False):
        self.x = x
        self.y = y
        self.z = z
        self.is_open = is_open

    def add(self, wp):
        self.x += wp.x
        self.y += wp.y
        self.z += wp.z

    def mul(self, scale):
        self.x *= scale
        self.y *= scale
        self.z *= scale

datas = {
    "T" : { "list" : [Waypoint(x=3.0, is_open=True),Waypoint(x=-1.5),Waypoint(z=-3.0, is_open=True)],
            "width" : 3.5},
    "E" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(x=2.0, is_open=True), Waypoint(z=1.5), Waypoint(x=-2.0, is_open=True), Waypoint(z=1.5), Waypoint(x=2.0, is_open=True)],
            "width" : 2.5},
    "K" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(2.0), Waypoint(x=-2.0, z=1.5, is_open=True), Waypoint(x=2.0, z=1.5, is_open=True)],
            "width" : 2.5},
    "N" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(2.0), Waypoint(x=-2.0 ,z=3.0, is_open=True), Waypoint(x=2.0), Waypoint(z=-3.0, is_open=True)],
            "width" : 2.5},
    "O" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(2.0, is_open=True), Waypoint(z=3.0, is_open=True), Waypoint(-2.0, is_open=True)],
            "width" : 2.5},
    "F" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(z=2.0), Waypoint(x=2.0, is_open=True), Waypoint(z=1.0), Waypoint(x=-2.0, is_open=True)],
            "width" : 2.5},
    "S" : { "list" : [Waypoint(z=-3.0), Waypoint(2.0, is_open=True), Waypoint(z=1.5, is_open=True), Waypoint(-2.0, is_open=True), Waypoint(z=1.5, is_open=True), Waypoint(2.0, is_open=True)], 
            "width" : 2.5},
    "2" : { "list" : [Waypoint(2.0, is_open=True), Waypoint(z=-1.5, is_open=True), Waypoint(-2.0, is_open=True), Waypoint(z=-1.5, is_open=True), Waypoint(2.0, is_open=True)],
            "width" : 2.5},
    "0" : { "list" : [Waypoint(z=-3.0, is_open=True), Waypoint(2.0, is_open=True), Waypoint(z=3.0, is_open=True), Waypoint(-2.0, is_open=True)],
            "width" : 2.5}
}

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