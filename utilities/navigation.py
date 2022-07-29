from utilities.utils import *

class MixinNavigation:
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

    def run_mission(self, vel=0.3):
        for wp in self.wps:
            self.is_open = wp.is_open
            self.cruise_control(wp.x, wp.y, wp.z)
    
    def print_path(self):
        if not len(self.wps):
            pass
        for wp in self.wps:
            print("({}, {}, {})".format(wp.x, wp.y, wp.z))
    
    def yazi_yaz(self, distance_to_wall):
        self.duvara_bak(distance_to_wall)
        sentence = input("Please enter the sentence to be painted on the wall\n")
        self.get_mission(sentence)
        self.run_mission()
        rospy.logdebug("mission completed !")