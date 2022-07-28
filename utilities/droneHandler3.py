from utilities.droneHandler2 import DroneHandler2
from utilities.utils import *
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
        (x, y) = target_loc = self.transform(x, y)
        x += self.x
        y += self.y
        self.move_global(x, y, self.z, 180 / math.pi * self.yaw + yaw)

    def is_target_reached(self, x, y, z, yaw, tolerance_lin=0.2, tolerance_ang=0.2):
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
        return x, y

    def get_mission(self, sentence: str):
        del self.wps
        self.wps = []

        prev_wp = Waypoint(self.x, self.y, self.z)
        for c in sentence:
            total_height = 0.0
            total_width = 0.0
            wp_lst = datas[c]["list"]
            box_width = datas[c]["width"]
            print("box width : ", str(box_width * self.config.font_size))
            for wp in wp_lst:
                new_wp = Waypoint(wp.x, wp.y, wp.z, wp.is_open)
                (new_wp.x, new_wp.y) = self.transform(new_wp.x, 0.0)
                new_wp.mul(self.config.font_size)
                new_wp.add(prev_wp)
                self.wps.append(new_wp)
                prev_wp = new_wp
                total_height += wp.z
                total_width += wp.x
            new_wp = Waypoint(x=(box_width - total_width), z=-total_height)
            (new_wp.x, new_wp.y) = self.transform(new_wp.x, 0.0)
            new_wp.mul(self.config.font_size)
            new_wp.add(prev_wp)
            self.wps.append(new_wp)
            prev_wp = new_wp

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

    def duvara_bak(self, distance=5.0):
        vel = 0
        count = 0
        prev_front = self.front
        self.set_vel_global(yaw_vel=0.1)
        while True:
            time.sleep(0.5)
            self.print_pose()
            if self.front > 12.0:
                self.set_vel_global(yaw_vel=self.config.max_yaw_vel)
                continue
            vel = self.config.kp_yaw * (prev_front - self.front) / prev_front
            prev_front = self.front
            if abs(vel) < 0.005:
                count += 1
            if count > 1:
                self.set_vel_global(yaw_vel=0.0)
                break
            if self.config.max_yaw_vel < vel:
                vel = self.config.max_yaw_vel
            elif -self.config.max_yaw_vel > vel:
                vel = -self.config.max_yaw_vel
            self.set_vel_global(yaw_vel=vel)
        self.move_local(y=(self.front - distance))

    def cruise_control(self, config):
        yaw_vel = 0
        prev_front = self.front
        if self.front > 12.0:
            yaw_vel = config.max_yaw_vel
        yaw_vel = config.kp_yaw * (prev_front - self.front) / prev_front
        prev_front = self.front
        if config.max_yaw_vel < yaw_vel:
            yaw_vel = config.max_yaw_vel
        elif -config.max_yaw_vel > yaw_vel:
            yaw_vel = -config.max_yaw_vel
        
        return yaw_vel

    def run_mission(self, vel=0.3):
        for wp in self.wps:
            self.is_open = wp.is_open
            self.cruise_control(wp.x, wp.y, wp.z)
    
    def yazi_yaz(self, distance_to_wall):
        self.duvara_bak(distance_to_wall)
        sentence = input("Please enter the sentence to be painted on the wall\n")
        self.get_mission(sentence)
        self.run_mission()
        rospy.logdebug("mission completed !")
