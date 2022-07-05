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
from mavros_python_examples.utils import *

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
        self.target_distance_to_wall = 2.0
        self.kp_linear_ver = 0.2
        self.kp_linear_hor = 0.5
        self.kp_angular = 2.5
        self.MAX_YAW_VEL = 0.5
        self.sim_lidar_mode = True
        self.real_lidar_mode = False
        self.odom_ready = False

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
        self.TOPIC_SET_LIN_ANG_VEL = TopicService("/mavros/setpoint_velocity/cmd_vel_unstamped", geometry_msgs.msg.Twist)
        self.TOPIC_GET_POSE_GLOBAL = TopicService("/mavros/global_position/local", nav_msgs.msg.Odometry)
        self.TOPIC_GET_VEL = TopicService("/mavros/local_position/velocity_body", geometry_msgs.msg.TwistStamped)
        self.TOPIC_SCAN = TopicService("/scan", sensor_msgs.msg.LaserScan)
        self.TOPIC_SIM_SCAN = TopicService("/spur/laser/scan", sensor_msgs.msg.LaserScan)

        self.thread_param_updater = threading.Timer(0, self.update_parameters_from_topic)
        self.thread_param_updater.daemon = True
        self.thread_param_updater.start()

    def enable_topics_for_read(self):
        self.topic_subscriber(self.TOPIC_STATE)
        self.topic_subscriber(self.TOPIC_GET_POSE_GLOBAL)
        self.topic_subscriber(self.TOPIC_GET_VEL)
        if self.sim_lidar_mode:
            self.topic_subscriber(self.TOPIC_SIM_SCAN)
        if self.real_lidar_mode:
            self.topic_subscriber(self.TOPIC_SCAN)

    def arm(self, status: bool):
        data = mavros_msgs.srv.CommandBoolRequest()
        data.value = status
        self.SERVICE_ARM.set_data(data)
        result = self.service_caller(self.SERVICE_ARM, timeout=30)
        return result.success

    def takeoff_internal(self, altitude: float):
        data = mavros_msgs.srv.CommandTOLRequest()
        data.min_pitch = 0.0
        data.yaw = 0.0
        data.latitude = 0.0
        data.longitude = 0.0
        data.altitude = altitude
        self.SERVICE_TAKEOFF.set_data(data)
        result = self.service_caller(self.SERVICE_TAKEOFF, timeout=30)
        return result.success

    def takeoff(self, altitude=3.0, tolerance=0.03):
        self.takeoff_internal(altitude)
        while self.z < altitude - tolerance:
            print(str(self.z))
            self.rate.sleep()

    def land_internal(self):
        data = mavros_msgs.srv.CommandTOLRequest()
        data.altitude = 0.0
        data.latitude = 0.0
        data.longitude = 0.0
        data.min_pitch= 0.0
        data.yaw = 0.0
        self.SERVICE_LAND.set_data(data)
        result = self.service_caller(self.SERVICE_LAND, timeout=30)
        return result.success

    def land(self):
        self.land_internal()
        while self.z > 0.05:
            print(str(self.z))
            self.rate.sleep()

    def change_mode(self, mode: str):
        data = mavros_msgs.srv.SetModeRequest()
        data.custom_mode = mode
        self.SERVICE_SET_MODE.set_data(data)
        result = self.service_caller(self.SERVICE_SET_MODE, timeout=30)
        return result.mode_sent

    def move_safe(self, gx=0.0, gy=0.0, gz=1.0, robot_type=RobotType.circle):
        print("Started !")
        self.move_global(self.x, self.y, gz)
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
            self.move_global(x[0], x[1], self.z, x[2])
            # check reaching goal
            dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
            if dist_to_goal <= config.robot_radius:
                print("Goal!!")
                break

    def print_ranges(self):
        for i in range(self.laser_count):
            angle = 180 / math.pi * (-math.pi + i * self.angle_increment)
            print(f"({angle}, {self.ranges[i]})")

    def range_for_angle(self, angle: float):
        return self.ranges[int(float(self.laser_count) * (angle / 360 + 0.5))]

    def initial_align(self, tolerance_ang=0.005):
        while True:
            print("yaw = ", str(180 / math.pi * self.yaw))
            left = self.range_for_angle(5.0)
            right = self.range_for_angle(-5.0)
            print(f"right = {right}")
            print(f"left = {left}")
            if (right > 12.0) or (left > 12.0):
                print("abc")
                if right <= 12.0:
                    self.set_vel_global(yaw_vel=-self.MAX_YAW_VEL)
                else:
                    self.set_vel_global(yaw_vel=self.MAX_YAW_VEL)
            else:
                diff = right - left
                if abs(diff) < tolerance_ang:
                    break
                elif diff * self.kp_angular > self.MAX_YAW_VEL:
                    self.set_vel_global(yaw_vel=self.MAX_YAW_VEL)
                elif diff * self.kp_angular < -self.MAX_YAW_VEL:
                    self.set_vel_global(yaw_vel=-self.MAX_YAW_VEL)
                else:
                    self.set_vel_global(yaw_vel=(self.kp_angular * diff))
            self.rate.sleep()

        mesafe = self.range_for_angle(0.0)
        self.move_local(y=(mesafe - self.target_distance_to_wall))

    def cruise_control(self, x, y, z, vel, tolerance_ang=0.005, tolerance_lin=0.05, tolerance_distance=0.3):
        xprime = 0.0
        yprime = 0.0
        y_prime_local = 0.0
        zprime = 0.0
        yaw_vel = 0.0
        temp_wp = Waypoint()
        left = self.range_for_angle(5.0)
        right = self.range_for_angle(-5.0)
        front = self.range_for_angle(0.0)
        if (left > 12.0) or (right > 12.0):
            return False
        diff_ang = right - left
        if abs(diff_ang) > tolerance_ang:
            if diff_ang * self.kp_angular > self.MAX_YAW_VEL:
                yaw_vel += self.MAX_YAW_VEL
            elif diff_ang * self.kp_angular < -self.MAX_YAW_VEL:
                yaw_vel += -self.MAX_YAW_VEL
            else:
                yaw_vel +=self.kp_angular * diff_ang

        diff_lin = self.target_distance_to_wall - front

        if abs(diff_lin) > tolerance_lin:
            yprime_local = -(diff_lin * self.kp_linear_ver)
            temp_wp.y = yprime_local
            temp_wp = self.transform(temp_wp)
            xprime += temp_wp.x
            yprime += temp_wp.y

        diff_x = x - self.x
        diff_y = y - self.y
        diff_z = z - self.z
        length = math.sqrt(math.pow(diff_x, 2) + math.pow(diff_y, 2) + math.pow(diff_z, 2))
        diff_x /= length;diff_y /= length;diff_z /= length
        
        if length > tolerance_distance:
            xprime += diff_x * vel
            yprime += diff_y * vel
            zprime += diff_z * vel
        else:
            ratio = length / tolerance_distance
            xprime += diff_x * vel * ratio * self.kp_linear_hor
            yprime += diff_y * vel * ratio * self.kp_linear_hor
            zprime += diff_z * vel * ratio * self.kp_linear_hor

        self.set_vel_global(xprime, yprime, zprime, yaw_vel)
        return True


    def move_global_internal(self, x, y, z, yaw):
        data = geometry_msgs.msg.PoseStamped()
        data.header.stamp = rospy.Time.now()
        data.pose.position.x = x
        data.pose.position.y = y
        data.pose.position.z = z
        (data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,
        data.pose.orientation.w) = get_quaternion_from_euler(self.roll, self.pitch, yaw)
        self.TOPIC_SET_POSE_GLOBAL.set_data(data)
        self.topic_publisher(topic=self.TOPIC_SET_POSE_GLOBAL)

    def set_vel_global(self, xprime=0.0, yprime=0.0, zprime=0.0, yaw_vel=0.0):
        data = geometry_msgs.msg.Twist()
        data.linear.x = xprime
        data.linear.y = yprime
        data.linear.z = zprime
        data.angular.z = yaw_vel
        data.angular.y = 0.0
        data.angular.x = 0.0
        self.TOPIC_SET_LIN_ANG_VEL.set_data(data)
        self.topic_publisher(topic=self.TOPIC_SET_LIN_ANG_VEL)

    def set_vel_local(self, xprime=0.0, yprime=0.0, zprime=0.0, yaw_vel=0.0):
        target_vel = self.transform(Waypoint(xprime, yprime, zprime))
        self.set_vel_global(target_vel.x, target_vel.y, target_vel.z, yaw_vel)

    def move_global(self, x=None, y=None, z=None, yaw=None):
        if not x:
            x = self.x
        if not y:
            y = self.y
        if not z:
            z = self.z
        if not yaw:
            yaw = self.yaw
        else:
            yaw = angle2radian(yaw)
            print(str(yaw))
        while not rospy.is_shutdown():
            self.move_global_internal(x, y, z, yaw)
            self.print_vel()
            self.print_pose()
            if self.is_target_reached(x, y, z, yaw):
                break
            self.rate.sleep()

    def is_target_reached(self, x, y, z, yaw, tolerance_lin=0.1, tolerance_ang=0.1):
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
        print("----------------------------")
        print("X is : ", str(self.x))
        print("Y is : ", str(self.y))
        print("Z is : ", str(self.z))
        print("Yaw is : ", str(180 / math.pi * self.yaw))

    def print_vel(self):
        print("----------------------------")
        print("x_prime : ", str(self.xprime))
        print("y_prime : ", str(self.yprime))
        print("z_prime : ", str(self.zprime))
        print("yaw_vel = ", str(self.yaw_vel))

    def get_param(self, param: str):
        data = mavros_msgs.srv.ParamGetRequest()
        data.param_id = param
        self.SERVICE_GET_PARAM.set_data(data)
        result = self.service_caller(self.SERVICE_GET_PARAM, timeout=30)
        return result.success, result.value.integer, result.value.real

    def set_param(self, param: str, value_integer=0, value_real=0.0):
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

    def transform(self, wp: Waypoint): # not finished
        yaw = self.yaw - math.pi / 2
        tempy = wp.y
        tempx = wp.x
        wp.x = math.cos(yaw) * tempx - math.sin(yaw) * tempy
        wp.y = math.cos(yaw) * tempy + math.sin(yaw) * tempx
        wp.z = wp.z
        return wp

    def move_local(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        local_diff = Waypoint(x, y, z)
        target_loc = self.transform(local_diff)
        target_loc.x += self.x
        target_loc.y += self.y
        target_loc.z += self.z
        self.move_global(target_loc.x, target_loc.y, target_loc.z, 180 / math.pi * self.yaw + yaw)

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
                new_wp = self.transform(self.copy(wp))
                new_wp.mul(self.k)
                new_wp.add(prev_wp)
                self.wps.append(new_wp)
                prev_wp = new_wp
                total_height += wp.z
                total_width += wp.x
            new_wp = self.transform(Waypoint(x=(box_width - total_width), z=-total_height))
            new_wp.mul(self.k)
            new_wp.add(prev_wp)
            self.wps.append(new_wp)
            prev_wp = new_wp

    def go_to_waypoint(self, x, y, z, vel):
        while not rospy.is_shutdown():
            if not self.cruise_control(x, y, z, vel):
                return False
            self.print_vel()
            self.print_pose()
            print("is nozzle open", str(self.is_open))
            print("mesafe = ", str(self.range_for_angle(0.0)))
            if self.is_target_reached(x, y, z, self.yaw):
                return True
            self.rate.sleep()

    def run_mission(self, vel=0.3):
        for wp in self.wps:
            self.is_open = wp.is_open
            if (self.sim_lidar_mode or self.real_lidar_mode):
                if not self.go_to_waypoint(wp.x, wp.y, wp.z, vel):
                    break
            else:
                self.move_global(wp.x, wp.y, wp.z)

    def print_path(self):
        if not len(self.wps):
            pass
        for wp in self.wps:
            print("({}, {}, {})".format(wp.x, wp.y, wp.z))

    def update_parameters_from_topic(self):
        while True:
            if not self.odom_ready and (self.TOPIC_STATE.get_data() != None
                and self.TOPIC_GET_POSE_GLOBAL.get_data() != None
                and self.TOPIC_GET_VEL.get_data() != None):
                self.odom_ready = True
            time.sleep(1)
            if not self.odom_ready:
                continue
            if (self.real_lidar_mode or self.sim_lidar_mode):
                if self.real_lidar_mode and self.TOPIC_SCAN.get_data():
                    break
                elif self.sim_lidar_mode and self.TOPIC_SIM_SCAN.get_data():
                    break
            else:
                break

        if self.real_lidar_mode:
            self.angle_increment = self.TOPIC_SCAN.get_data().angle_increment
            self.laser_count = int(math.pi * 2 / self.angle_increment)
        elif self.sim_lidar_mode:
            self.angle_increment = self.TOPIC_SIM_SCAN.get_data().angle_increment
            self.laser_count = int(math.pi * 2 / self.angle_increment)
        while True:
            if self.connected:
                state_data = self.TOPIC_STATE.get_data()
                pose_data = self.TOPIC_GET_POSE_GLOBAL.get_data()
                vel_data = self.TOPIC_GET_VEL.get_data()
                if self.sim_lidar_mode:
                    scan_data = self.TOPIC_SIM_SCAN.get_data()
                elif self.real_lidar_mode:
                    scan_data = self.TOPIC_SCAN.get_data()
                self.mode = state_data.mode
                self.armed = state_data.armed
                self.x = pose_data.pose.pose.position.x
                self.y = pose_data.pose.pose.position.y
                self.z = pose_data.pose.pose.position.z
                self.xprime = vel_data.twist.linear.x
                self.yprime = vel_data.twist.linear.y
                self.zprime = vel_data.twist.linear.z
                self.yaw_vel = vel_data.twist.angular.z
                w = pose_data.pose.pose.orientation.w
                x = pose_data.pose.pose.orientation.x
                y = pose_data.pose.pose.orientation.y
                z = pose_data.pose.pose.orientation.z
                (self.roll, self.pitch, self.yaw) = quaternion_to_euler(w, x, y, z)
                self.roll = angle2radian(self.roll); self.pitch = angle2radian(self.pitch); self.yaw = angle2radian(self.yaw);
                if self.real_lidar_mode:
                    self.ranges = scan_data.ranges
                elif self.sim_lidar_mode:
                    self.ranges = scan_data.ranges
                self.rate.sleep()
