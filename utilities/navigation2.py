from utilities.utils import *

class MixinNavigation2:
    def duvara_bak(self, distance=5.0):
        self.config.distance = distance
        vel = 0.0
        count = 0
        self.is_init = False
        while True:
            time.sleep(0.5)
            self.print_pose()
            if self.front > 40.0:
                self.is_init = False
                vel = self.config.max_yaw_vel
            elif not self.is_init:
                self.prev_front = self.front
                vel=0.1
                self.is_init = True
            else:
                vel = self.config.kp_yaw * (self.prev_front - self.front) / self.prev_front
                self.prev_front = self.front
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
        self.is_init = False
    
    def init_wall(self, wall:Wall, distance=5.0):
        self.duvara_bak(distance=distance)
        self.go_most_down()
        print("bitti")
        self.go_most_right()
        print("bitti")
        (x1, y1, z1) = (self.x, self.y, self.z)
        self.go_most_up()
        print("bitti")
        self.go_most_left()
        print("bitti")
        (x2, y2, z2) = (self.x, self.y, self.z)
        wall.height = z2 - z1
        wall.width = get_distance(x1, y1, z1, x2, y2, z2)
        wall.angle = math.atan2(y1 - y2, x1 - x2)
        wall.is_init = True

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

    def get_mission(self):
        del self.wps
        self.wps = []

        prev_wp = Waypoint(self.x, self.y, self.z)
        wall = self.wall
        sentence = wall.sentence
        for c in sentence:
            total_height = 0.0
            total_width = 0.0
            wp_lst = datas[c]["list"]
            box_width = datas[c]["width"]
            print("box width : ", str(box_width * self.config.font_scale))
            for wp in wp_lst:
                new_wp = Waypoint(wp.x, wp.y, wp.z, wp.is_open)
                (new_wp.x, new_wp.y) = self.transform(new_wp.x, 0.0)
                new_wp.mul(self.config.font_scale)
                new_wp.add(prev_wp)
                self.wps.append(new_wp)
                prev_wp = new_wp
                total_height += wp.z
                total_width += wp.x
            new_wp = Waypoint(x=(box_width - total_width), z=-total_height)
            (new_wp.x, new_wp.y) = self.transform(new_wp.x, 0.0)
            new_wp.mul(self.config.font_scale)
            new_wp.add(prev_wp)
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
        config = self.config
        for c in self.wall.sentence:
            total_height = 0.0
            total_width = 0.0
            wp_list = datas[c]["list"]
            box_width = datas[c]["width"]
            for wp in wp_list:
                rospy.loginfo("writing " + c)
                total_height += wp.z * config.font_scale
                total_width += wp.x * config.font_scale
                self.is_open = wp.is_open
                self.move_local(x=(wp.x*config.font_scale), z=(wp.z*config.font_scale))
            self.is_open = False
            self.move_local(x=(box_width - total_width), z=-total_height)
    
    def run_mission_with_vel(self, vel=0.3):
        self.wall.sentence = input("Type the sentence.\n")
        self.get_mission()
        for wp in self.wps:
            print("is_open : ", wp.is_open)
            self.move_global_with_vel(wp.x, wp.y, wp.z, vel=vel)
            time.sleep(0.3)

    def print_path(self):
        if not len(self.wps):
            pass
        for wp in self.wps:
            print("({}, {}, {})".format(wp.x, wp.y, wp.z))
    
    def yazi_yaz(self, distance_to_wall):
        self.init_wall(self.wall, distance_to_wall)
        self.wall.sentence = input("Please enter the sentence to be painted on the wall\n")
        self.run_mission()
        rospy.logdebug("mission completed !")
        self.wall.is_init = False