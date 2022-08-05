from utilities.service_handling import angle2radian, quaternion_to_euler
from utilities.utils import *


class MixinSubscribing:
    def home_pose_cb(self, data):
        self.home[0] = data.geo.latitude
        self.home[1] = data.geo.longitude
        self.home[2] = data.geo.altitude

    def pose_rel_global_cb(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        w = data.pose.pose.orientation.w
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        (self.roll, self.pitch, self.yaw) = quaternion_to_euler(w, x, y, z)
        self.roll = angle2radian(self.roll)
        self.pitch = angle2radian(self.pitch)
        self.yaw = angle2radian(self.yaw)

    def pose_global_cb(self, data):
        self.altitude = data.altitude
        self.longitude = data.longitude
        self.latitude = data.latitude

    def vel_global_cb(self, data):
        self.xprime = data.twist.linear.x
        self.yprime = data.twist.linear.y
        self.zprime = data.twist.linear.z
        self.yaw_vel = data.twist.angular.z
    
    def state_cb(self, data):
        self.current_mode = data.mode
        self.is_armed = data.armed

    def lidar_sim_cb(self, data):
        angle = angle2radian(5.0)
        self.right = data.ranges[int((math.pi - angle) / data.angle_increment)]
        self.left = data.ranges[int((math.pi + angle) / data.angle_increment)]


    def print_pose(self):
        print("----------------------------")
        print("X is : ", str(self.x))
        print("Y is : ", str(self.y))
        print("Z is : ", str(self.z))
        print("Yaw is : ", str(180 / math.pi * self.yaw))
        print("front : ", self.get_front())

    def print_pose_global(self):
        print("------------------------------")
        print("latitude : ", self.latitude)
        print("longitude : ", self.longitude)
        print("altitude : ", self.altitude)

    def print_vel(self):
        print("----------------------------")
        print("x_prime : ", str(self.xprime))
        print("y_prime : ", str(self.yprime))
        print("z_prime : ", str(self.zprime))
        print("yaw_vel = ", str(self.yaw_vel))