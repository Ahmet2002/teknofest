from mavros_python_examples.droneHandler1 import DroneHandler1, angle2radian, quaternion_to_euler
from mavros_python_examples.utils import *


class DroneHandler2(DroneHandler1):
    def __init__(self):
        super().__init__()
        self.sub_pose_global = rospy.Subscriber("/mavros/global_position/local", nav_msgs.msg.Odometry, self.pose_global_cb)
        self.sub_vel_global = rospy.Subscriber("/mavros/local_position/velocity_body", geometry_msgs.msg.TwistStamped, self.vel_global_cb)
        self.sub_state = rospy.Subscriber("/mavros/state", mavros_msgs.msg.State, self.state_cb)


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