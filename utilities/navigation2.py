from utilities.utils import *
from simple_pid import PID

class MixinNavigation2:
    def duvara_bak(self, distance=5.0):
        self.config.distance = distance
        vel = 0.0
        while not rospy.is_shutdown():
            self.print_pose()
            if (self.left > 40.0) or (self.right > 40.0):
                if self.right <= 40.0:
                    vel = -self.config.max_yaw_vel
                else:
                    vel = self.config.max_yaw_vel
            else:
                vel = self.config.kp_yaw * (self.right - self.left) / self.left
                if abs(vel) < 0.01:
                    break
                if self.config.max_yaw_vel < vel:
                    vel = self.config.max_yaw_vel
                elif -self.config.max_yaw_vel > vel:
                    vel = -self.config.max_yaw_vel
            self.set_vel_global(yaw_vel=vel)
            self.rate.sleep()
        self.move_local(y=(self.get_front() - distance))

    def duvara_bak_deneme(self, distance=0.5):
        pid_yaw = PID(Kp=0.5, Ki=0.2, Kd=0.2, setpoint=0.0, sample_time=0.1)
        pid.output_limits = (-0.3, 0.3)
        self.config.distance = distance
        diff = 0.0
        control = 0.0
        while not rospy.is_shutdown():
            self.print_pose()
            if (self.left > 40.0) or (self.right > 40.0):
                if self.right <= 40.0:
                    vel = -self.config.max_yaw_vel
                else:
                    vel = self.config.max_yaw_vel
            else:
                diff = self.left - self.right
                if abs(diff) < 0.01:
                    break
                control = pid(diff)
            self.set_vel_global(yaw_vel=control)
            self.rate.sleep()
        self.move_local(y=(self.get_front() - distance))

    def go_2d_on_wall(self, x=0.0, y=0.0):
        wall = self.wall
        if not wall.is_init:
            rospy.logerr("Error on go_2d_on_wall() function.")
            rospy.logerr("wall has not been initialized yet.")
            return
        if not wall.in_borders(x=x, y=y):
            rospy.logerr("out of walls borders!")
            return
        (target_x, target_y, target_z) = wall.get_exact_loc(x=x, y=y, transform=self.transform)
        self.move_global_safe(target_x, target_y, target_z)

    def get_new_from_prev(self, prev_wp:Waypoint, dx=0.0, dy=0.0, dz=0.0, is_open=False):
        new_wp = Waypoint(x=dx, y=dy, z=dz, is_open=is_open)
        (new_wp.x, new_wp.y) = self.transform(new_wp.x, 0.0)
        new_wp.mul(self.config.font_scale)
        tmp_x = prev_wp.x + (180.0/math.pi)*(new_wp.y/6378137)
        new_wp.y = prev_wp.y + (180.0/math.pi)*(new_wp.x/6378137)/math.cos(math.pi/180.0*prev_wp.x)
        new_wp.x = tmp_x
        new_wp.z += prev_wp.z
        return new_wp

    def get_mission(self):
        del self.wps
        self.wps = []

        prev_wp = Waypoint(self.latitude, self.longitude, self.altitude - self.home[2])
        wall = self.wall
        sentence = wall.sentence
        for c in sentence:
            total_height = 0.0
            total_width = 0.0
            wp_lst = datas[c]["list"]
            box_width = datas[c]["width"]
            print("box width : ", str(box_width * self.config.font_scale))
            for wp in wp_lst:
                new_wp = self.get_new_from_prev(prev_wp, wp.x, wp.y, wp.z, wp.is_open)
                self.wps.append(new_wp)
                prev_wp = new_wp
                total_height += wp.z
                total_width += wp.x
            new_wp = self.get_new_from_prev(prev_wp, dx=(box_width - total_width), dz=-total_height)
            self.wps.append(new_wp)
            prev_wp = new_wp

    def run_mission(self, vel=0.3):
        config = self.config
        for c in self.wall.sentence:
            total_height = 0.0
            total_width = 0.0
            wp_list = datas[c]["list"]
            box_width = datas[c]["width"]
            for wp in wp_list:
                total_height += wp.z * config.font_scale
                total_width += wp.x * config.font_scale
                self.is_open = wp.is_open
                self.move_local_safe(x=(wp.x*config.font_scale), z=(wp.z*config.font_scale), vel=vel)
            self.is_open = False
            self.move_local_safe(x=(box_width - total_width), z=-total_height, vel=vel)

    def run_mission_without_lidar(self):
        self.wall.sentence = input("Type the sentence.\n")
        self.get_mission()
        for wp in self.wps:
            self.is_open = wp.is_open
            print("is_open : ", wp.is_open)
            self.move_global(wp.x, wp.y, wp.z, self.yaw)
            time.sleep(0.2)

    def print_path(self):
        if not len(self.wps):
            pass
        for wp in self.wps:
            print("({}, {}, {})".format(wp.x, wp.y, wp.z))

    def init_wall(self, distance=5.0):
        self.duvara_bak(distance=distance)
        self.go_most_up()
        print("bitti")
        self.go_most_left()
        print("bitti")
    
    def yazi_yaz(self, distance_to_wall):
        # self.init_wall(distance_to_wall)
        self.duvara_bak(distance=distance_to_wall)
        self.wall.sentence = input("Please enter the sentence to be painted on the wall\n")
        self.run_mission()
        rospy.logdebug("mission completed !")