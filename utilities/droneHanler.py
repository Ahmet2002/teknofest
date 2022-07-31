import rospy
from utilities.service import Service
from utilities.utils import *
from utilities.service_handling import MixinServiceHandler
from utilities.rospyHandler import MixinRosHandler
from utilities.navigation import MixinNavigation
from utilities.navigation2 import MixinNavigation2
from utilities.publishing import MixinPublishing
from utilities.subscribing import MixinSubscribing

class DroneHandler(MixinServiceHandler, MixinRosHandler, MixinNavigation, MixinPublishing, MixinSubscribing, MixinNavigation2):
    def __init__(self):
        self.is_armed = False
        self.current_mode = ""
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.latitude = 0.0
        self.altitude = 0.0
        self.longitude = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.xprime = 0.0
        self.yprime = 0.0
        self.zprime = 0.0
        self.yaw_vel = 0.0
        self.config = Config()
        self.wall = Wall()
        self.front = 0.0
        self.prev_front = 0.0
        self.wps = []
        self.front_history = []
        self.frequency = 5
        self.connected = False
        self.is_init = False

        # Services
        self.service_arm = Service("/mavros/cmd/arming", mavros_msgs.srv.CommandBool)
        self.service_mission_push = Service("/mavros/mission/push", mavros_msgs.srv.WaypointPush)
        self.service_takeoff = Service("/mavros/cmd/takeoff", mavros_msgs.srv.CommandTOL)
        self.service_land = Service("/mavros/cmd/land", mavros_msgs.srv.CommandTOL)
        self.service_mission_clear = Service("/mavros/mission/clear", mavros_msgs.srv.WaypointClear)
        self.service_set_mode = Service("/mavros/set_mode", mavros_msgs.srv.SetMode)
        self.service_set_param = Service("/mavros/param/set", mavros_msgs.srv.ParamSet)
        self.service_get_param = Service("/mavros/param/get", mavros_msgs.srv.ParamGet)
        
        # Publishers
        self.pub_vel_global = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", geometry_msgs.msg.Twist, queue_size=10)
        self.pub_pose_global = rospy.Publisher('/mavros/setpoint_position/local', geometry_msgs.msg.PoseStamped, queue_size=10)

        # Subscribers
        self.sub_pose_global = rospy.Subscriber("/mavros/global_position/local", nav_msgs.msg.Odometry, self.pose_rel_global_cb)
        self.sub_pose_global = rospy.Subscriber("/mavros/global_position/global", sensor_msgs.msg.NavSatFix, self.pose_global_cb)
        self.sub_vel_global = rospy.Subscriber("/mavros/local_position/velocity_body", geometry_msgs.msg.TwistStamped, self.vel_global_cb)
        self.sub_state = rospy.Subscriber("/mavros/state", mavros_msgs.msg.State, self.state_cb)
        self.sub_lidar = rospy.Subscriber("/spur/laser/scan", sensor_msgs.msg.LaserScan, self.lidar_cb)