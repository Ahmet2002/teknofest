from rospy import is_shutdown
from utilities.utils import *
from simple_pid import PID

class MixinNavigation:
    def duvara_bak(self, distance=5.0):
        pid_yaw = PID(Kp=0.3, Ki=0.2, Kd=1.0, setpoint=0.0, sample_time=0.1)
        pid_yaw.output_limits = (-0.3, 0.3)
        self.config.distance = distance
        diff = 0.0
        control = 0.0
        while not rospy.is_shutdown():
            self.print_pose()
            if (self.left > 40.0) or (self.right > 40.0):
                if self.right <= 40.0:
                    control = -self.config.max_yaw_vel
                else:
                    control = self.config.max_yaw_vel
            else:
                diff = self.left - self.right
                print("diff : ", diff)
                if abs(diff) < 0.005 * self.get_front():
                    self.fixed_yaw = self.yaw
                    break
                control = pid_yaw(diff)
            self.set_vel_global(yaw_vel=control)
            self.rate.sleep()
        self.move_local_safe(y=(self.get_front() - distance))

    def get_front(self):
        return (self.right * math.cos(self.angle_offset))
        
    def move_global_safe(self, x:float, y:float, z:float, vel=0.2):
        config = self.config
        pid_x = PID(Kp=0.5, Ki=0.2, Kd=0.4, setpoint=x, sample_time=0.1)
        pid_y = PID(Kp=0.5, Ki=0.2, Kd=0.4, setpoint=y, sample_time=0.1)
        pid_z = PID(Kp=0.5, Ki=0.2, Kd=0.4, setpoint=z, sample_time=0.1)
        pid_yaw = PID(Kp=0.2, Ki=0.1, Kd=0.3, setpoint=self.fixed_yaw, sample_time=0.1)
        pid_front = PID(Kp=0.3, Ki=0.1, Kd=0.2, setpoint=config.distance, sample_time=0.1)
        pid_x.output_limits = (-vel, vel)
        pid_y.output_limits = (-vel, vel)
        pid_z.output_limits = (-vel, vel)
        pid_front.output_limits = (-vel, vel)
        pid_yaw.output_limits = (-0.2, 0.2)
        while not rospy.is_shutdown() and not self.is_target_reached(x, y, z, check_yaw=False):
            print("target_x : ", x)
            print("target_y : ", y)
            print("target_z : ", z)
            self.print_pose()
            (vel_x, vel_y, vel_z) = (pid_x(self.x), pid_y(self.y), pid_z(self.z))

            # Compute yaw vel
            if math.isinf(self.front):
                rospy.logerr("out of wall !")
                return False
            else:
                yaw_vel = pid_yaw(self.yaw)

            control = pid_front(self.front)
            (x_vel, y_vel) = self.transform(0.0, control)
            vel_x += x_vel
            vel_y += y_vel
            self.set_vel_global(vel_x, vel_y, vel_z, yaw_vel)
            self.rate.sleep()
        
        return True

    def move_local_safe(self,x=0.0, y=0.0, z=0.0, vel=0.2):
        (x, y) = self.transform(x, y)
        (x, y, z) = (x + self.x, y + self.y, z + self.z)
        if self.move_global_safe(x, y, z, vel=vel):
            return True
        return False

    def go_most_left(self):
        while not rospy.is_shutdown():
            if not self.move_local_safe(x=-1.0):
                break
        self.move_local(x=1.5)
    
    def go_most_right(self):
        while not rospy.is_shutdown():
            if not self.move_local_safe(x=1.0):
                break
        self.move_local(x=-1.5)

    def go_most_up(self):
        while not rospy.is_shutdown():
            if not self.move_local_safe(z=1.0):
                break
        self.move_local(z=-1.5)

    def go_most_down(self):
        while not rospy.is_shutdown():
            if not self.move_local_safe(z=-1.0):
                break
        self.move_local(z=1.5)

    def init_wall(self, distance=5.0):
        self.duvara_bak(distance=distance)
        self.go_most_up()
        print("en yukariya gitti")
        self.go_most_left()
        print("En sola gitti")

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

    def run_mission(self, fixed_yaw, distance, vel=0.2):
        self.aciyi_ve_uzakligi_ayarla(fixed_yaw=fixed_yaw, distance=distance)
        config = self.config
        config.distance = distance
        sentence = input("Type the sentence.\n")
        self.wall.sentence = sentence.upper().strip()
        for c in self.wall.sentence:
            total_height = 0.0
            total_width = 0.0
            wp_list = datas[c]["list"]
            box_width = datas[c]["width"]
            for wp in wp_list:
                total_height += wp.z * config.font_scale
                total_width += wp.x * config.font_scale
                self.is_open = wp.is_open
                if not self.move_local_safe(x=(wp.x*config.font_scale), z=(wp.z*config.font_scale), vel=vel):
                    self.change_mode(MODE_RTL)
                    self.disconnect()
            self.is_open = False
            if not self.move_local_safe(x=(box_width - total_width), z=-total_height, vel=vel):
                self.change_mode(MODE_RTL)
                self.disconnect()

    def yazi_yaz(self, distance_to_wall):
        # self.init_wall(distance_to_wall)
        self.init_wall(distance=distance_to_wall)
        self.wall.sentence = input("Please enter the sentence to be painted on the wall\n")
        self.run_mission()
        rospy.logdebug("mission completed !")
