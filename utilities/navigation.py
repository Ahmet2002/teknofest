from rospy import is_shutdown
from utilities.utils import *
from simple_pid import PID

class MixinNavigation:
    def get_front(self):
        return (self.right * math.cos(self.angle_offset))
        
    def move_global_safe(self, x:float, y:float, z:float, vel=0.15):
        config = self.config
        pid_x = PID(Kp=0.5, Ki=0.2, Kd=0.4, setpoint=x, sample_time=0.1)
        pid_y = PID(Kp=0.5, Ki=0.2, Kd=0.4, setpoint=y, sample_time=0.1)
        pid_z = PID(Kp=0.5, Ki=0.2, Kd=0.4, setpoint=z, sample_time=0.1)
        pid_yaw = PID(Kp=0.5, Ki=0.0, Kd=1.0, setpoint=0.0, sample_time=0.1)
        pid_x.output_limits = (-vel, vel)
        pid_y.output_limits = (-vel, vel)
        pid_z.output_limits = (-vel, vel)
        pid_yaw.output_limits = (-0.2, 0.2)
        while not rospy.is_shutdown() and not self.is_target_reached(x, y, z, check_yaw=False):
            print("target_x : ", x)
            print("target_y : ", y)
            print("target_z : ", z)
            self.print_pose()
            (vel_x, vel_y, vel_z) = (pid_x(self.x), pid_y(self.y), pid_z(self.z))

            # Compute yaw vel
            if (self.left > 40.0) or (self.right > 40.0):
                rospy.logerr("out of wall !")
                return False
            else:
                yaw_vel = pid_yaw(self.left - self.right)
            
            self.set_vel_global(vel_x, vel_y, vel_z, yaw_vel)
            self.rate.sleep()
        
        return True

    def move_local_safe(self,x=0.0, y=0.0, z=0.0):
        (x, y) = self.transform(x, y)
        (x, y, z) = (x + self.x, y + self.y, z + self.z)
        if self.move_global_safe(x, y, z):
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
