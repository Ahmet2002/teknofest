from mavros_python_examples.droneHandler2 import DroneHandler2
from mavros_python_examples.utils import *
import rospy

class DroneHandler3(DroneHandler2):

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
        target_vel = self.transform(Waypoint(xprime, yprime, zprime))
        self.set_vel_global(target_vel.x, target_vel.y, target_vel.z, yaw_vel)

    def move_global_internal(self, x, y, z, yaw):
        data = geometry_msgs.msg.PoseStamped()
        data.header.stamp = rospy.Time.now()
        data.pose.position.x = x
        data.pose.position.y = y
        data.pose.position.z = z
        (data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,
        data.pose.orientation.w) = get_quaternion_from_euler(self.roll, self.pitch, yaw)
        self.pub_pose_global.publish(data)

    def move_global(self, x=None, y=None, z=None, yaw=None):
        yaw = angle2radian(yaw)
        while not rospy.is_shutdown():
            self.move_global_internal(x, y, z, yaw)
            self.print_vel()
            self.print_pose()
            if self.is_target_reached(x, y, z, yaw):
                break
            self.rate.sleep()

    def move_local(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        local_diff = Waypoint(x, y, z)
        target_loc = self.transform(local_diff)
        target_loc.x += self.x
        target_loc.y += self.y
        target_loc.z += self.z
        self.move_global(target_loc.x, target_loc.y, target_loc.z, 180 / math.pi * self.yaw + yaw)

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

    def transform(self, x:float, y:float): # not finished
        yaw = self.yaw - math.pi / 2
        tmp_x = x
        x = math.cos(yaw) * tmp_x - math.sin(yaw) * y
        y = math.cos(yaw) * y + math.sin(yaw) * tmp_x

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
                new_wp = Waypoint(wp.x, wp.y, wp.z, wp.is_open)
                new_wp = self.transform(new_wp.x, new_wp.y)
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

    def run_mission(self, vel=0.3):
        for wp in self.wps:
            self.is_open = wp.is_open
            self.move_global(wp.x, wp.y, wp.z)

    def print_path(self):
        if not len(self.wps):
            pass
        for wp in self.wps:
            print("({}, {}, {})".format(wp.x, wp.y, wp.z))

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