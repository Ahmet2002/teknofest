from utilities.utils import *

class MixinNavigation:
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
            x_vel += x
            y_vel += y
        return x_vel, y_vel, z_vel, yaw_vel

    def move_global_safe(self, x:float, y:float, z:float, vel=0.3):
        config = self.config
        while not rospy.is_shutdown() and not self.is_target_reached(x, y, z, self.yaw):
            self.print_pose()
            d = get_distance(self.x, self.y, self.z, x, y, z)
            (dx, dy, dz) = (x-self.x, y-self.y, z-self.z)
            if d > config.min_distance:
                (vel_x, vel_y, vel_z) = (dx*vel/d, dy*vel/d, dz*vel/d)
            else:
                (vel_x, vel_y, vel_z) = (dx*config.kp_nav, dy*config.kp_nav, dz*config.kp_nav)
            self.set_controlled_vel_global(x_vel=vel_x, y_vel=vel_y, z_vel=vel_z)
            self.rate.sleep()
        self.is_init = False
    
    def move_global_with_vel(self, x:float, y:float, z:float, yaw=None, vel=0.3):
        if not yaw:
            check_yaw = False
        config = self.config
        while not rospy.is_shutdown() and not self.is_target_reached(x, y, z, self.yaw, check_yaw=False):
            d = get_distance(self.x, self.y, self.z, x, y, z)
            (dx, dy, dz) = (x-self.x, y-self.y, z-self.z)
            if d > config.min_distance:
                (vel_x, vel_y, vel_z) = (dx*vel/d, dy*vel/d, dz*vel/d)
            else:
                (vel_x, vel_y, vel_z) = (dx*config.kp_nav, dy*config.kp_nav, dz*config.kp_nav)
            self.set_vel_global(vel_x, vel_y, vel_z)

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
            self.print_pose()
            self.set_controlled_vel(x_vel=vel)
            self.rate.sleep()
        self.set_vel_global()
        self.move_local(x=-1.0)
        self.is_init = False

    def go_most_left(self, vel=0.3):
        while self.front < 40.0:
            self.print_pose()
            self.set_controlled_vel(x_vel=-vel)
            self.rate.sleep()
        self.set_vel_global()
        self.move_local(x=1.0)
        self.is_init = False

    def go_most_up(self, vel=0.3):
        while self.front < 40.0:
            self.print_pose()
            self.set_controlled_vel(z_vel=vel)
            self.rate.sleep()
        self.set_vel_global()
        self.move_local(z=-1.0)
        self.is_init = False

    def go_most_down(self, vel=0.3):
        while (self.front < 40.0) and (self.z > 2.0):
            self.print_pose()
            self.set_controlled_vel(z_vel=-vel)
            self.rate.sleep()
        self.set_vel_global()
        self.move_local(z=1.0)
        self.is_init = False
