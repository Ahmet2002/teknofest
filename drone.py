import threading
import time
from utilities.droneHanler import DroneHandler
from utilities.utils import *

GPIO.setmode(GPIO.BOARD)
GPIO.setup(33,GPIO.OUT)

class MyDroneHandler(DroneHandler):
    def __init__(self):
        super().__init__()

        self.user_thread = threading.Timer(0, self.user)
        self.user_thread.daemon = True
        self.user_thread.start()

    def user(self):
        while not self.connected:
            rospy.loginfo("Waiting to be connected.")
            time.sleep(1)

        print("WPNAV_SPEED SET : ", str(self.set_param(param="WPNAV_SPEED", value_real=30.0)).upper())
        print("WPNAV_SPEED_UP SET : ", str(self.set_param(param="WPNAV_SPEED_UP", value_real=30.0)).upper())
        print("WPNAV_SPEED_DN SET : ", str(self.set_param(param="WPNAV_SPEED_DN", value_real=30.0)).upper())
        print("WPNAV_RADIUS SET : ", str(self.set_param(param="WPNAV_RADIUS", value_real=20.0)).upper())
        print("WPNAV_ACCEL SET : ", str(self.set_param(param="WPNAV_ACCEL", value_real=50.0)).upper())
        print("WPNAV_ACCEL_Z SET : ", str(self.set_param(param="WPNAV_ACCEL_Z", value_real=50.0)).upper())
        print("WP_YAW_BEHAVIOR SET : ", str(self.set_param(param="WP_YAW_BEHAVIOR", value_real=0.0)).upper())


        self.takeoff(4.5)
        time.sleep(2.0)

        # self.run_mission_with_lidar_word_to_word(distance=2.4)
        # # self.run_mission_with_lidar(3.14, 2.0)
        # # self.run_mission(3.14, 1.5)
        self.run_mission_without_lidar()

        self.land()
        # self.change_mode(MODE_RTL)
        self.disconnect()




if __name__ == "__main__":
    v = MyDroneHandler()
    v.connect("node1", rate=10)

