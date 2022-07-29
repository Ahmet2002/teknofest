import time
import rospy
from utilities.utils import *

class MixinServiceHandler:
    def arm(self, status: bool):
        data = mavros_msgs.srv.CommandBoolRequest()
        data.value = status
        self.service_arm.set_data(data)
        result = self.service_caller(self.service_arm)
        return result.success

    def takeoff(self, altitude=5.0, tolerance=0.1):
        if self.current_mode != MODE_GUIDED:
            rospy.loginfo("Drone is not in GUIDED mode\nChanging to GUIDED, " + str(self.change_mode(MODE_GUIDED)))
        if not self.is_armed and not self.arm(True):
            rospy.logerr("Arm error")
            exit(1)
        data = mavros_msgs.srv.CommandTOLRequest()
        data.min_pitch = 0.0
        data.yaw = 0.0
        data.latitude = 0.0
        data.longitude = 0.0
        data.altitude = altitude
        self.service_takeoff.set_data(data)
        result = self.service_caller(self.service_takeoff)
        if not result.success:
            rospy.logerr("Takeoff error")
            return False
        altitude -= tolerance
        while self.z < altitude:
            print(str(self.z))
            time.sleep(1)

    def land(self):
        data = mavros_msgs.srv.CommandTOLRequest()
        data.altitude = 0.0
        data.latitude = 0.0
        data.longitude = 0.0
        data.min_pitch= 0.0
        data.yaw = 0.0
        self.service_land.set_data(data)
        result = self.service_caller(self.service_land, timeout=30)
        if not result.success:
            rospy.logerr("Landing Error !")
        while self.z > 0.05:
            print(str(self.z))
            self.rate.sleep()

    def change_mode(self, mode: str):
        data = mavros_msgs.srv.SetModeRequest()
        data.custom_mode = mode
        self.service_set_mode.set_data(data)
        result = self.service_caller(self.service_set_mode, timeout=30)
        return result.mode_sent


    def get_param(self, param: str):
        data = mavros_msgs.srv.ParamGetRequest()
        data.param_id = param
        self.service_get_param.set_data(data)
        result = self.service_caller(self.service_get_param, timeout=30)
        return result.success, result.value.integer, result.value.real

    def set_param(self, param="WPNAV_SPEED", value_real=0.0):
        data = mavros_msgs.srv.ParamSetRequest()
        data.param_id = param
        data.value.real = value_real
        self.service_get_param.set_data(data)
        result = self.service_caller(self.service_set_param, timeout=30)
        return result.success