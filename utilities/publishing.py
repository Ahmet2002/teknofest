from utilities.utils import *
import rospy

class MixinPublishing:

    def set_vel_global(self, xprime=0.0, yprime=0.0, zprime=0.0, yaw_vel=0.0):
        data = geometry_msgs.msg.Twist()
        data.linear.x = xprime
        data.linear.y = yprime
        data.linear.z = zprime
        data.angular.z = yaw_vel
        data.angular.y = 0.0
        data.angular.x = 0.0
        self.pub_vel_global.publish(data)

    def set_vel_local(self, xprime=0.0, yprime=0.0, zprime=0.0, yaw_vel=0.0):
        (xprime, yprime) = self.transform(xprime, yprime)
        self.set_vel_global(xprime, yprime, zprime, yaw_vel)

    def move_global_internal(self, x, y, z, yaw):
        data = geometry_msgs.msg.PoseStamped()
        data.header.stamp = rospy.Time.now()
        data.pose.position.x = x
        data.pose.position.y = y
        data.pose.position.z = z
        (data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,
        data.pose.orientation.w) = get_quaternion_from_euler(self.roll, self.pitch, yaw)
        self.pub_pose_global.publish(data)

    def move_global(self, x=None, y=None , z=None, yaw=None):
        if x == None:
            x = self.x
        if y == None:
            y = self.y
        if z == None:
            z = self.z
        if yaw == None:
            yaw = self.yaw
        else:
            yaw = angle2radian(yaw)
        while not rospy.is_shutdown():
            self.move_global_internal(x, y, z, yaw)
            self.print_vel()
            self.print_pose()
            if self.is_target_reached(x, y, z, yaw):
                break
            self.rate.sleep()

    def move_local(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        (x, y) = self.transform(x, y)
        x += self.x
        y += self.y
        z += self.z
        self.move_global(x, y, z, 180 / math.pi * self.yaw + yaw)

    def is_target_reached(self, x, y, z, yaw, check_yaw=True, tolerance_lin=0.2, tolerance_ang=0.2):
        dx = self.x - x
        dy = self.y - y
        dz = self.z - z
        
        distance = math.sqrt((math.pow(dx, 2) + math.pow(dy, 2) + math.pow(dz, 2)))
        if (distance <= tolerance_lin):
            if check_yaw:
                dyaw = self.yaw - yaw
                dyaw = abs(math.atan2(math.sin(dyaw), math.cos(dyaw)))
                if dyaw > tolerance_ang:
                    return False
            return True
        return False

    def transform(self, x:float, y:float, angle=None): # not finished
        if not angle:
            angle= self.yaw - math.pi / 2
        tmp_x = x
        x = math.cos(angle) * tmp_x - math.sin(angle) * y
        y = math.cos(angle) * y + math.sin(angle) * tmp_x
        return x, y
