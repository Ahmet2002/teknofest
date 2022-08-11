import time
import rospy
from utilities.utils import *
from mavros_msgs.msg import Waypoint

class MixinServiceHandler:
    def arm(self, status: bool):
        data = mavros_msgs.srv.CommandBoolRequest()
        data.value = status
        self.service_arm.set_data(data)
        result = self.service_caller(self.service_arm)
        return result.success

    def takeoff(self, altitude=5.0, tolerance=0.2):
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
        while self.z > 0.3:
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

    def set_param(self, param, value_real):
        data = mavros_msgs.srv.ParamSetRequest()
        data.param_id = param
        data.value.real = value_real
        self.service_set_param.set_data(data)
        result = self.service_caller(self.service_set_param, timeout=30)
        return result.success

    def push_wps(self):
        data = mavros_msgs.srv.WaypointPushRequest()
        data.start_index = 0
        data.waypoints = self.wps
        self.service_mission_push.set_data(data)
        result = self.service_caller(self.service_mission_push, timeout=30)
        return result.success

    def clear_wps(self):
        data = mavros_msgs.srv.WaypointClearRequest()
        self.service_mission_clear.set_data(data)
        result = self.service_caller(self.service_mission_clear, timeout=30)
        del self.wps
        self.wps = []
        return result.success
        
    def add_wp_to_wp_list(self,frame,command,is_current,autocontinue,param1,param2,param3,param4,x_lat,y_long,z_alt):
        wp = Waypoint()
        wp.frame =frame #  FRAME_GLOBAL_REL_ALT = 3 for more visit http://docs.ros.org/api/mavros_msgs/html/msg/Waypoint.html
        wp.command = command #VTOL TAKEOFF = 84,NAV_WAYPOINT = 16, TAKE_OFF=22 for checking out other parameters go to https://github.com/mavlink/mavros/blob/master/mavros_msgs/msg/CommandCode.msg
        wp.is_current= is_current
        wp.autocontinue = autocontinue # enable taking and following upcoming waypoints automatically 
        wp.param1=param1 # no idea what these are for but the script will work so go ahead
        wp.param2=param2
        wp.param3=param3
        wp.param4=param4
        wp.x_lat= x_lat 
        wp.y_long=y_long
        wp.z_alt= z_alt #relative altitude.
        self.wps.append(wp)