import threading
import time

import mavros_msgs.msg
import mavros_msgs.srv
import nav_msgs.msg
import geometry_msgs.msg
import math
import rospy

class Waypoint:
    def __init__(self, x=0.0, y=0.0, z=0.0, is_open=False):
        self.x = x
        self.y = y
        self.z = z
        self.is_open = is_open

datas = {
    "T" : [Waypoint(y=50.0), Waypoint(x=75.0, is_open=True),Waypoint(x=-37.5),Waypoint(y=-50.0, is_open=True)]
}

from mavros_python_examples.rospyHandler import RosHandler
from mavros_python_examples.topicService import TopicService

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
        self.xprime = 0.0
        self.yprime = 0.0
        self.zprime = 0.0
        self.yaw_vel = 0.0
        self.box_width = 4.0
        self.wps = []

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
        self.TOPIC_GET_VEL = TopicService("/mavros/local_pition/velocity_body", geometry_msgs.msg.TwistStamped)


        self.thread_param_updater = threading.Timer(0, self.update_parameters_from_topic)
        self.thread_param_updater.daemon = True
        self.thread_param_updater.start()

    def enable_topics_for_read(self):
        self.topic_subscriber(self.TOPIC_STATE)
        self.topic_subscriber(self.TOPIC_GET_POSE_GLOBAL)

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
        data.altitude = 0
        data.latitude = 0
        data.longitude = 0
        self.SERVICE_LAND.set_data(data)
        result = self.service_caller(self.SERVICE_LAND, timeout=30)
        return result.success

    def change_mode(self, mode: str):
        data = mavros_msgs.srv.SetModeRequest()
        data.custom_mode = mode
        self.SERVICE_SET_MODE.set_data(data)
        result = self.service_caller(self.SERVICE_SET_MODE, timeout=30)
        return result.mode_sent

    def move(self, target: Waypoint):
        data = geometry_msgs.msg.PoseStamped()
        #data.header.stamp = rospy.Time.now()
        data.pose.position.x = target.x
        data.pose.position.y = target.y
        data.pose.position.z = target.z
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

    def move2target(self, x=0.0, y=0.0, z=3.0):
        target_pose = Waypoint(x, y, z)
        while not rospy.is_shutdown():
            self.move(target_pose)
            self.print_pose()
            if self.is_target_reached(target_pose):
                break
            self.rate.sleep()

    def is_target_reached(self, target: Waypoint, tolerance=0.2):
        dx = self.x - target.x
        dy = self.y - target.y
        dz = self.z - target.z
        distance = math.sqrt((math.pow(dx, 2) + math.pow(dy, 2) + math.pow(dz, 2)))
        if distance <= tolerance:
            return True
        return False

    def print_pose(self):
        print("X is : ", str(self.x))
        print("Y is : ", str(self.y))
        print("Z is : ", str(self.z))

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
        current_wp.x += prev_wp.x
        current_wp.y += prev_wp.y
        current_wp.z += prev_wp.z
        return current_wp

    def get_mission(self, sentence: str): # Not finished
        del self.wps
        self.wps = []

        prev_wp = Waypoint(self.x, self.y, self.z)
        for c in sentence:
            total_height = 0.0
            total_width = 0.0
            for wp in datas[c]:
                new_wp = self.transform(self.copy(wp), prev_wp)
                self.wps.append(new_wp)
                prev_wp = new_wp
                total_height += wp.y
                total_width += wp.x
            new_wp = self.transform(Waypoint(self.box_width - total_width, -total_height), prev_wp)
            self.wps.append(new_wp)
            prev_wp = new_wp

    def run_mission(self):
        for wp in self.wps:
            self.is_open = wp.is_open
            self.move2target(wp.x, wp.y, wp.z);

    def update_parameters_from_topic(self):
        while not (self.TOPIC_GET_POSE_GLOBAL.get_data() 
            and self.TOPIC_STATE.get_data()):
            time.sleep(1)
        while True:
            if self.connected:
                state_data = self.TOPIC_STATE.get_data()
                geo_data = self.TOPIC_GET_POSE_GLOBAL.get_data()
                self.mode = state_data.mode
                self.armed = state_data.armed
                self.x = geo_data.pose.pose.position.x
                self.y = geo_data.pose.pose.position.y
                self.z = geo_data.pose.pose.position.z
                self.xprime = geo_data.twist.twist.linear.x
                self.yprime = geo_data.twist.twist.linear.y
                self.zprime = geo_data.twist.twist.linear.z
                self.rate.sleep()
