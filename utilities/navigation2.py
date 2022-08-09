from utilities.utils import *
from simple_pid import PID

class MixinNavigation2:

    def __get_new_from_prev(self, prev_wp:Waypoint, dx=0.0, dy=0.0, dz=0.0, is_open=False):
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
                new_wp = self.__get_new_from_prev(prev_wp, wp.x, wp.y, wp.z, wp.is_open)
                self.wps.append(new_wp)
                prev_wp = new_wp
                total_height += wp.z
                total_width += wp.x
            new_wp = self.__get_new_from_prev(prev_wp, dx=(box_width - total_width), dz=-total_height)
            self.wps.append(new_wp)
            prev_wp = new_wp

    def print_path(self):
        if not len(self.wps):
            pass
        for wp in self.wps:
            print("({}, {}, {})".format(wp.x, wp.y, wp.z))

    def run_mission_without_lidar(self, fixed_yaw):
        self.move_global(self.latitude, self.longitude, self.altitude - self.home[2], fixed_yaw)
        self.wall.sentence = input("Type the sentence.\n")
        self.get_mission()
        for wp in self.wps:
            self.is_open = wp.is_open
            print("is_open : ", wp.is_open)
            self.move_global(wp.x, wp.y, wp.z, fixed_yaw)
            time.sleep(0.2)