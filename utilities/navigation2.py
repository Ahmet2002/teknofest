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

    def get_mission(self, sentence):
        del self.wps
        self.wps = []
        wall = self.wall

        prev_wp = Waypoint(self.latitude, self.longitude, self.altitude - self.home[2])
        for c in sentence:
            total_height = 0.0
            total_width = 0.0
            wp_lst = wall.chars[c]["list"]
            box_width = wall.chars[c]["width"]
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

    def __aciyi_ve_uzakligi_ayarla(self):
        self.move_global(lat=self.latitude, lon=self.longitude, alt=(self.altitude - self.home[2], self.fixed_yaw))
        while math.isinf(self.front):
            self.set_vel_local(xprime=0.1)
            self.rate.sleep()
        self.move_local(y=(self.front - self.config.distance))
        self.move_global(lat=self.latitude, lon=self.longitude, alt=(self.altitude - self.home[2], self.fixed_yaw))
        print("aci ve uzaklik ayarlandi, hazir.")

    def __yeni_satira_gec(self):
        origin = self.wall.origin
        self.move_global(origin.x, origin.y, origin.z, self.fixed_yaw)
        self.move_local(z=-((self.wall.satir_araligi + 3.0)*self.config.font_scale))
        self.__aciyi_ve_uzakligi_ayarla()
        time.sleep(0.5)

    def __satiri_ortala(self, word):
        total_width = 0.0
        for c in word:
            total_width += self.wall.chars[c]["width"]
        offset = (self.wall.width - 1.0 - total_width * self.config.font_scale) / 2
        self.move_local(x=offset)
        self.__aciyi_ve_uzakligi_ayarla()
        time.sleep(0.5)
    
    def __check_sentence(self, words):
        satir_sayisi = len(words)
        total_height = satir_sayisi * 3.0 + (satir_sayisi - 1) * self.wall.satir_araligi * self.config.font_scale
        if total_height > (self.wall.width - 1.0):
            return False
        for word in words:
            total_width = 0.0
            for c in word:
                total_width += self.wall.chars[c]["width"]
            if total_width > self.wall.width:
                return False
        return True


    def run_mission_with_lidar_wp2wp(self, distance):
        config = self.config
        wall = self.wall
        config.distance = distance
        self.__aciyi_ve_uzakligi_ayarla()
        wall.origin = Point(self.latitude, self.longitude, self.altitude - self.home[2])
        sentence = input("Type the sentence.\n")
        wall.words = sentence.upper().strip().split(",")
        while not self.__check_sentence(words=wall.words):
            print("Sentence doesn't fit to wall.\n please try again with another one")
            sentence = input("Type the sentence.\n")
            wall.words = sentence.upper().strip().split(",")

        word_count = len(wall.words)
        for i in range(word_count):
            self.__satiri_ortala(word=wall.words[i])
            for c in wall.words[i]:
                total_height = 0.0
                total_width = 0.0
                wp_list = wall.chars[c]["list"]
                box_width = wall.chars[c]["width"]
                for wp in wp_list:
                    total_height += wp.z
                    total_width += wp.x
                    self.is_open = wp.is_open
                    # if self.is_open:
                    #     nozzle_on()
                    # else:
                    #     nozzle_off()
                    self.move_local(x=(wp.x*config.font_scale), z=(wp.z*config.font_scale))
                    # nozzle_off()
                    self.__aciyi_ve_uzakligi_ayarla()
                    time.sleep(0.5)
                self.is_open = False
                self.move_local(x=(box_width - total_width)*config.font_scale, z=-total_height*config.font_scale)
                self.__aciyi_ve_uzakligi_ayarla()
                time.sleep(0.5)
            if i != word_count - 1:
                self.__yeni_satira_gec()

    def run_mission_with_lidar_word_to_word(self, distance):
        config = self.config
        wall = self.wall
        config.distance = distance
        self.__aciyi_ve_uzakligi_ayarla()
        wall.origin = Point(self.latitude, self.longitude, self.altitude - self.home[2])
        sentence = input("Type the sentence.\n")
        wall.words = sentence.upper().strip().split(",")
        while not self.__check_sentence(words=wall.words):
            print("Sentence doesn't fit to wall.\n please try again with another one")
            sentence = input("Type the sentence.\n")
            wall.words = sentence.upper().strip().split(",")
        word_count = len(wall.words)
        for i in range(word_count):
            self.__satiri_ortala(word=wall.words[i])
            for c in wall.words[i]:
                self.get_mission(sentence=c)
                for wp in self.wps:
                    self.is_open = wp.is_open
                    # if self.is_open:
                    #     nozzle_on()
                    # else:
                    #     nozzle_off()
                    self.move_global(lat=wp.x, lon=wp.y, alt=wp.z, yaw=self.fixed_yaw)
                    # nozzle_off()
                    time.sleep(0.5)
                self.__aciyi_ve_uzakligi_ayarla()
            if i != word_count - 1:
                self.__yeni_satira_gec()
            

    def run_mission_with_lidar(self, fixed_yaw, distance):
        self.move_global(self.latitude, self.longitude, self.altitude - self.home[2], fixed_yaw)
        self.move_local(y=(self.front - distance))
        self.move_global(self.latitude, self.longitude, self.altitude - self.home[2], fixed_yaw)
        sentence = input("Type the sentence.\n")
        self.wall.sentence = sentence.upper().strip()
        self.get_mission(sentence=self.wall.sentence)
        for wp in self.wps:
            self.is_open = wp.is_open
            # if self.is_open:
            #     nozzle_on()
            # else:
            #     nozzle_off()
            print("is_open : ", str(wp.is_open))
            self.move_global(wp.x, wp.y, wp.z, fixed_yaw)
            # nozzle_off()
            time.sleep(0.7)

    def run_mission_without_lidar(self, fixed_yaw):
        sentence = input("Type the sentence.\n")
        self.wall.sentence = sentence.upper().strip()
        self.get_mission(sentence=self.wall.sentence)
        for wp in self.wps:
            self.is_open = wp.is_open
            # if self.is_open:
            #     nozzle_on()
            # else:
            #     nozzle_off()
            print("is_open : ", str(wp.is_open))
            self.move_global(wp.x, wp.y, wp.z, yaw=fixed_yaw)
            # nozzle_off()
            time.sleep(0.7)