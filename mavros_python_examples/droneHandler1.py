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
from mavros_python_examples.dynamic_window_approach import *
from mavros_python_examples.rospyHandler import RosHandler
from mavros_python_examples.service import Service
from mavros_python_examples.utils import *

class DroneHandler1(RosHandler):
    def __init__(self):
        super().__init__()
        self.is_armed = False
        self.current_mode = ""
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.xprime = 0.0
        self.yprime = 0.0
        self.zprime = 0.0
        self.yaw_vel = 0.0
        self.k = 1.0
        self.wps = []

        
        self.service_arm = Service("/mavros/cmd/arming", mavros_msgs.srv.CommandBool)
        self.service_mission_push = Service("/mavros/mission/push", mavros_msgs.srv.WaypointPush)
        self.service_takeoff = Service("/mavros/cmd/takeoff", mavros_msgs.srv.CommandTOL)
        self.service_land = Service("/mavros/cmd/land", mavros_msgs.srv.CommandTOL)
        self.service_mission_clear = Service("/mavros/mission/clear", mavros_msgs.srv.WaypointClear)
        self.service_set_mode = Service("/mavros/set_mode", mavros_msgs.srv.SetMode)
        self.service_set_param = Service("/mavros/param/set", mavros_msgs.srv.ParamSet)
        self.service_get_param = Service("/mavros/param/get", mavros_msgs.srv.ParamGet)
        self.pub_vel_global = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", geometry_msgs.msg.Twist, queue_size=10)
        self.pub_pose_global = rospy.Publisher('/mavros/setpoint_position/local', geometry_msgs.msg.PoseStamped, queue_size=10)

    def arm(self, status: bool):
        data = mavros_msgs.srv.CommandBoolRequest()
        data.value = status
        self.service_arm.set_data(data)
        result = self.service_caller(self.service_arm)
        return result.success

    def takeoff(self, altitude=5.0, tolerance=0.3):
        if self.current_mode != MODE_GUIDED:
            rospy.loginfo("Drone is not in GUIDED mode\nChanging to GUIDED, " + str(self.change_mode(MODE_GUIDED)))
        if not self.is_armed and not self.arm(True):
            rospy.logerr("Arm error")
        data = mavros_msgs.srv.CommandTOLRequest()
        data.min_pitch = 0.0
        data.yaw = 0.0
        data.latitude = 0.0
        data.longitude = 0.0
        data.altitude = altitude
        self.service_takeoff.set_data(data)
        result = self.service_caller(self, self.service_takeoff)
        if not result.success:
            rospy.logerr("Takeoff error")
            return False
        altitude -= tolerance
        while self.z < altitude:
            print(str(self.z))
            time.sleep(1)

    def land(self):
        data = mavros_msgs.srv.CommandTOLRequest()
        data.altitude = 0.0
        data.latitude = 0.0
        data.longitude = 0.0
        data.min_pitch= 0.0
        data.yaw = 0.0
        self.service_land.set_data(data)
        result = self.service_caller(self.service_land, timeout=30)
        if not result.success:
            rospy.logerr("Landing Error !")
        while self.z > 0.05:
            print(str(self.z))
            self.rate.sleep()

    def change_mode(self, mode: str):
        data = mavros_msgs.srv.SetModeRequest()
        data.custom_mode = mode
        self.service_set_mode.set_data(data)
        result = self.service_caller(self.service_set_mode, timeout=30)
        return result.mode_sent


    def get_param(self, param: str):
        data = mavros_msgs.srv.ParamGetRequest()
        data.param_id = param
        self.service_get_param.set_data(data)
        result = self.service_caller(self.service_get_param, timeout=30)
        return result.success, result.value.integer, result.value.real

    def set_param(self, param: str, value_integer=0, value_real=0.0):
        data = mavros_msgs.srv.ParamSetRequest()
        data.param_id = param
        data.value.integer = value_integer
        data.value.real = value_real
        self.service_get_param.set_data(data)
        result = self.service_caller(self.service_set_param, timeout=30)
        return result.success, result.value.integer, result.value.real