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
from mavros_python_examples.topicService import TopicService

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

class DroneHandler(RosHandler):
    def __init__(self):
        super().__init__()
        self.armed = False
        self.mode = ""
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
        self.ranges = []
        self.laser_count = 0
        self.angle_increment = 0.0

        self.TOPIC_STATE = TopicService("/mavros/state", mavros_msgs.msg.State)
        self.SERVICE_ARM = TopicService("/mavros/cmd/arming", mavros_msgs.srv.CommandBool)
        self.SERVICE_MISSION_PUSH = TopicService("/mavros/mission/push", mavros_msgs.srv.WaypointPush)
        self.SERVICE_TAKEOFF = TopicService("/mavros/cmd/takeoff", mavros_msgs.srv.CommandTOL)
        self.SERVICE_LAND = TopicService("/mavros/cmd/land", mavros_msgs.srv.CommandTOL)
        self.SERVICE_MISSION_CLEAR = TopicService("/mavros/mission/clear", mavros_msgs.srv.WaypointClear)
        self.SERVICE_SET_MODE = TopicService("/mavros/set_mode", mavros_msgs.srv.SetMode)
        self.SERVICE_SET_PARAM = TopicService("/mavros/param/set", mavros_msgs.srv.ParamSet)
        self.SERVICE_GET_PARAM = TopicService("/mavros/param/get", mavros_msgs.srv.ParamGet)
        self.TOPIC_SET_POSE_GLOBAL = TopicService('/mavros/setpoint_position/local', geometry_msgs.msg.PoseStamped)
        self.TOPIC_SET_LIN_ANG_VEL = TopicService("/mavros/setpoint_velocity/cmd_vel", geometry_msgs.msg.TwistStamped)
        self.TOPIC_GET_POSE_GLOBAL = TopicService("/mavros/global_position/local", nav_msgs.msg.Odometry)
        self.TOPIC_GET_VEL = TopicService("/mavros/local_position/velocity_body", geometry_msgs.msg.TwistStamped)
        self.TOPIC_SCAN = TopicService("/scan", sensor_msgs.msg.LaserScan)

        self.thread_param_updater = threading.Timer(0, self.update_parameters_from_topic)
        self.thread_param_updater.daemon = True
        self.thread_param_updater.start()

    def enable_topics_for_read(self):
        self.topic_subscriber(self.TOPIC_STATE)
        self.topic_subscriber(self.TOPIC_GET_POSE_GLOBAL)
        self.topic_subscriber(self.TOPIC_SCAN)

    def arm(self, status: bool):
        data = mavros_msgs.srv.CommandBoolRequest()
        data.value = status
        self.SERVICE_ARM.set_data(data)
        result = self.service_caller(self.SERVICE_ARM, timeout=30)
        return result.success

    def takeoff(self, altitude: float):
        data = mavros_msgs.srv.CommandTOLRequest()
        data.min_pitch = 0.0
        data.yaw = 0.0
        data.latitude = 0.0
        data.longitude = 0.0
        data.altitude = altitude
        self.SERVICE_TAKEOFF.set_data(data)
        result = self.service_caller(self.SERVICE_TAKEOFF, timeout=30)
        return result.success

    def land(self):
        data = mavros_msgs.srv.CommandTOLRequest()
        data.altitude = 0.0
        data.latitude = 0.0
        data.longitude = 0.0
        data.min_pitch= 0.0
        data.yaw = 0.0
        self.SERVICE_LAND.set_data(data)
        result = self.service_caller(self.SERVICE_LAND, timeout=30)
        return result.success

    def change_mode(self, mode: str):
        data = mavros_msgs.srv.SetModeRequest()
        data.custom_mode = mode
        self.SERVICE_SET_MODE.set_data(data)
        result = self.service_caller(self.SERVICE_SET_MODE, timeout=30)
        return result.mode_sent

    def move(self, x, y, z, yaw):
        data = geometry_msgs.msg.PoseStamped()
        #data.header.stamp = rospy.Time.now()
        data.pose.position.x = x
        data.pose.position.y = y
        data.pose.position.z = z
        (data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,
        data.pose.orientation.w) = get_quaternion_from_euler(self.roll, self.pitch, yaw)
        self.TOPIC_SET_POSE_GLOBAL.set_data(data)
        self.topic_publisher(topic=self.TOPIC_SET_POSE_GLOBAL)

    def set_vel(self, xprime=0.0, yprime=0.0, zprime=0.0, yaw_vel=0.0):
        data = geometry_msgs.msg.TwistStamped()
        data.twist.linear.x = xprime
        data.twist.linear.y = yprime
        data.twist.linear.z = zprime
        data.twist.angular.z = yaw_vel
        data.twist.angular.y = 0.0
        data.twist.angular.x = 0.0
        self.TOPIC_SET_LIN_ANG_VEL.set_data(data)
        self.topic_publisher(topic=self.TOPIC_SET_LIN_ANG_VEL)

    def move_safe(self, gx=0.0, gy=0.0, gz=1.0, robot_type=RobotType.circle):
        print("Started !")
        self.move2target(self.x, self.y, gz)
        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        x = np.array([self.x, self.y, self.yaw, math.hypot(self.xprime, self.yprime), self.yaw_vel])
        # goal position [x(m), y(m)]
        goal = np.array([gx, gy])

        # input [forward speed, yaw_rate]

        config.robot_type = robot_type
        trajectory = np.array(x)
        ob = config.ob
        while True:
            u, predicted_trajectory = dwa_control(x, config, goal, ob)
            x = motion(x, u, config.dt)  # simulate robot
            trajectory = np.vstack((trajectory, x))  # store state history
            self.move2target(x[0], x[1], self.z, x[2])
            # check reaching goal
            dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
            if dist_to_goal <= config.robot_radius:
                print("Goal!!")
                break

    def print_ranges(self):
        for i in range(self.laser_count):
            angle = 180 / math.pi * (-math.pi + i * self.angle_increment)
            print(f"({angle}, {self.ranges[i]})")

    def move2target(self, x=0.0, y=0.0, z=3.0, yaw=None):
        if not yaw:
            yaw = self.yaw
        while not rospy.is_shutdown():
            self.move(x, y, z, yaw)
            self.print_pose()
            if self.is_target_reached(x, y, z, yaw):
                break
            self.rate.sleep()

    def is_target_reached(self, x, y, z, yaw, tolerance_lin=1.0, tolerance_ang=0.7):
        dx = self.x - x
        dy = self.y - y
        dz = self.z - z
        dyaw = self.yaw - yaw
        dyaw = abs(math.atan2(math.sin(dyaw), math.cos(dyaw)))
        distance = math.sqrt((math.pow(dx, 2) + math.pow(dy, 2) + math.pow(dz, 2)))
        if (distance <= tolerance_lin) and (dyaw <= tolerance_ang):
            return True
        return False

    def print_pose(self):
        print("X is : ", str(self.x))
        print("Y is : ", str(self.y))
        print("Z is : ", str(self.z))
        print("Yaw is : ", str(self.yaw))

    def print_vel(self):
        print("x_prime : ", str(self.xprime))
        print("y_prime : ", str(self.yprime))
        print("z_prime : ", str(self.zprime))

    def get_param(self, param: str):
        data = mavros_msgs.srv.ParamGetRequest()
        data.param_id = param
        self.SERVICE_GET_PARAM.set_data(data)
        result = self.service_caller(self.SERVICE_GET_PARAM, timeout=30)
        return result.success, result.value.integer, result.value.real

    def set_param(self, param: str, value_integer: int, value_real: float):
        data = mavros_msgs.srv.ParamSetRequest()
        data.param_id = param
        data.value.integer = value_integer
        data.value.real = value_real
        self.SERVICE_SET_PARAM.set_data(data)
        result = self.service_caller(self.SERVICE_SET_PARAM, timeout=30)
        return result.success, result.value.integer, result.value.real

    @staticmethod
    def copy(wp: Waypoint):
        return Waypoint(wp.x, wp.y, wp.z, wp.is_open)

    def transform(self, current_wp: Waypoint, prev_wp: Waypoint): # not finished
        current_wp.y = current_wp.y * self.k + prev_wp.y
        current_wp.x = current_wp.x * self.k + prev_wp.x
        current_wp.z = current_wp.z * self.k + prev_wp.z
        return current_wp

    def get_mission(self, sentence: str):
        del self.wps
        self.wps = []

        prev_wp = Waypoint(self.x, self.y, self.z)
        for c in sentence:
            total_height = 0.0
            total_width = 0.0
            wp_lst = datas[c]["list"]
            box_width = datas[c]["width"]
            print("box width : ", str(box_width * self.k))
            for wp in wp_lst:
                new_wp = self.transform(self.copy(wp), prev_wp)
                self.wps.append(new_wp)
                prev_wp = new_wp
                total_height += wp.z
                total_width += wp.x
            new_wp = self.transform(Waypoint(x=(box_width - total_width), z=-total_height), prev_wp)
            self.wps.append(new_wp)
            prev_wp = new_wp

    def run_mission(self):
        for wp in self.wps:
            self.is_open = wp.is_open
            self.move2target(wp.x, wp.y, wp.z);

    def print_path(self):
        if not len(self.wps):
            pass
        for wp in self.wps:
            print("({}, {}, {})".format(wp.x, wp.y, wp.z))

    def update_parameters_from_topic(self):
        while not (self.TOPIC_GET_POSE_GLOBAL.get_data() 
            and self.TOPIC_STATE.get_data()):
            time.sleep(1)

        # self.angle_increment = self.TOPIC_SCAN.get_data().angle_increment
        # self.laser_count = int(math.pi * 2 / self.angle_increment)
        while True:
            if self.connected:
                state_data = self.TOPIC_STATE.get_data()
                geo_data = self.TOPIC_GET_POSE_GLOBAL.get_data()
                #scan_data = self.TOPIC_SCAN.get_data()
                self.mode = state_data.mode
                self.armed = state_data.armed
                self.x = geo_data.pose.pose.position.x
                self.y = geo_data.pose.pose.position.y
                self.z = geo_data.pose.pose.position.z
                self.xprime = geo_data.twist.twist.linear.x
                self.yprime = geo_data.twist.twist.linear.y
                self.zprime = geo_data.twist.twist.linear.z
                w = self.TOPIC_GET_POSE_GLOBAL.get_data().pose.pose.orientation.w
                x = self.TOPIC_GET_POSE_GLOBAL.get_data().pose.pose.orientation.x
                y = self.TOPIC_GET_POSE_GLOBAL.get_data().pose.pose.orientation.y
                z = self.TOPIC_GET_POSE_GLOBAL.get_data().pose.pose.orientation.z
                (self.roll, self.pitch, self.yaw) = quaternion_to_euler(w, x, y, z)
                self.roll = angle2radian(self.roll); self.pitch = angle2radian(self.pitch); self.yaw = angle2radian(self.yaw);
                # self.ranges = scan_data.ranges
                self.rate.sleep()
