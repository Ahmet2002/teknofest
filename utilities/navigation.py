from utilities.utils import *

class MixinNavigation:
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
        self.go_most_left()
        (x1, y1, z1) = (self.x, self.y, self.z)
        self.go_most_right()
        self.go_most_up()
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

    def combine_vels(self, x=0.0, y=0.0, z=0.0, is_global=True):
        config = self.config

        # Compute yaw vel
        if self.front > 40.0:
            self.is_init = False
            return None, None, None, None
        if not self.is_init:
            self.prev_front = self.front
            yaw_vel=0.1
            self.is_init = True
        else:
            yaw_vel = config.kp_yaw * (self.prev_front - self.front) / self.prev_front
            self.prev_front = self.front
            if config.max_yaw_vel < yaw_vel:
                yaw_vel = config.max_yaw_vel
            elif -config.max_yaw_vel > yaw_vel:
                yaw_vel = -config.max_yaw_vel
        
        # Compute front_vel to adjust the distance to wall
        y_vel = (self.front - config.distance) * config.kp_nav
        (x_vel, y_vel) = self.transform(0.0, y_vel)
        z_vel = z
        if is_global:
            x_vel += x
            y_vel += y
        else:
            (x, y) = self.transform(x, 0.0)
        return x_vel, y_vel, z_vel, yaw_vel

    def move_global_safe(self, x:float, y:float, z:float, vel=0.3):
        config = self.config
        while not rospy.is_shutdown() and not self.is_target_reached(x, y, z):
            d = get_distance(self.x, self.y, self.z, x, y, z)
            (dx, dy, dz) = (x-self.x, y-self.y, z-self.z)
            if d > config.min_distance:
                (vel_x, vel_y, vel_z) = (dx*vel/d, dy*vel/d, dz*vel/d)
            else:
                (vel_x, vel_y, vel_z) = (dx*config.kp_nav, dy*config.kp_nav, dz*config.kp_nav)
            self.set_controlled_vel_global(x_vel=vel_x, y_vel=vel_y, z_vel=vel_z)
        self.is_init = False    

    def move_local_safe(self,x=0.0, z=0.0, vel=0.3):
        (x, y) = self.transform(x, 0.0)
        (x, y, z) = (x + self.x, y + self.y, z + self.z)
        self.move_global_safe(x, y, z, vel=vel)

    def set_controlled_vel(self, x_vel=0.0, z_vel=0.0):
        (x_vel, y_vel, z_vel, yaw_vel) = self.combine_vels(x=x_vel, z=z_vel, is_global=False)
        if x_vel == None:
            pass
        self.set_vel_global(x_vel, y_vel, z_vel, yaw_vel)
    
    def set_controlled_vel_global(self, x_vel=0.0, y_vel=0.0, z_vel=0.0):
        (x_vel, y_vel, z_vel, yaw_vel) = self.combine_vels(x=x_vel, y=y_vel, z=z_vel)
        self.set_vel_global(x_vel, y_vel, z_vel, yaw_vel)

    def go_most_right(self, vel=0.3):
        while self.front < 40.0:
            self.set_controlled_vel(x_vel=-vel)
            self.rate.sleep()
        self.set_vel_global()
        self.move_local(x=0.25)
        self.is_init = False

    def go_most_left(self, vel=0.3):
        while self.front < 40.0:
            self.set_controlled_vel(x_vel=vel)
            self.rate.sleep()
        self.set_vel_global()
        self.move_local(x=-0.25)
        self.is_init = False

    def go_most_up(self, vel=0.3):
        while self.front < 40.0:
            self.set_controlled_vel(z_vel=vel)
            self.rate.sleep()
        self.set_vel_global()
        self.move_local(z=-0.25)
        self.is_init = False

    def go_most_down(self, vel=0.3):
        while (self.front < 40.0) and (self.z > 2.0):
            self.print_pose()
            self.set_controlled_vel(z_vel=-vel)
            self.rate.sleep()
        self.set_vel_global()
        self.move_local(z=0.25)
        self.is_init = False
    
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