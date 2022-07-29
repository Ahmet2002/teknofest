from utilities.service_handling import angle2radian, quaternion_to_euler
from utilities.utils import *


class MixinSubscribing:
    def pose_global_cb(self, data):
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

    def vel_global_cb(self, data):
        self.xprime = data.twist.linear.x
        self.yprime = data.twist.linear.y
        self.zprime = data.twist.linear.z
        self.yaw_vel = data.twist.angular.z
    
    def state_cb(self, data):
        self.current_mode = data.mode
        self.is_armed = data.armed

    def lidar_cb(self, data):
        self.front = data.ranges[int(math.pi / data.angle_increment)]