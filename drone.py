import threading
import time
from utilities.droneHanler import DroneHandler
from utilities.utils import *

# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(33,GPIO.OUT)

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

        print("WPNAV_SPEED SET : ", str(self.set_param(param="WPNAV_SPEED", value_real=500.0)).upper())
        print("WPNAV_SPEED_UP SET : ", str(self.set_param(param="WPNAV_SPEED_UP", value_real=500.0)).upper())
        print("WPNAV_SPEED_DN SET : ", str(self.set_param(param="WPNAV_SPEED_DN", value_real=500.0)).upper())
        print("WPNAV_RADIUS SET : ", str(self.set_param(param="WPNAV_RADIUS", value_real=30.0)).upper())
        print("WPNAV_ACCEL SET : ", str(self.set_param(param="WPNAV_ACCEL", value_real=50.0)).upper())
        print("WPNAV_ACCEL_Z SET : ", str(self.set_param(param="WPNAV_ACCEL_Z", value_real=50.0)).upper())
        self.takeoff(5.0)
        time.sleep(2.0)


        # self.duvara_bak(distance=2.0)
        # self.wall.sentence = input("Type the sentence.\n")
        # self.run_mission()
        # self.move_global_rel(yaw=0.0)
        self.run_mission_with_lidar_wp2wp(3.14, 2.0)
        # self.run_mission_without_lidar(3.14)
        # self.run_mission(3.14, 1.5)
        # self.change_mode(MODE_AUTO)
        # print(str(self.clear_wps()))
        # self.add_wp_to_wp_list(3,22,True,True,0.0,0.0,0.0,float('nan'), -35.36326767, 149.16524363, 50)
        # self.add_wp_to_wp_list(3,16,True,True,0.0,0.0,0.0,float('nan'), -35.36170368, 149.16500413, 50)
        # self.add_wp_to_wp_list(3,16,False,True,0.0,0.0,0.0,float('nan'), -35.36232276, 149.16337969, 50)
        # self.add_wp_to_wp_list(3,20,False,True,0.0,0.0,0.0,float('nan'), -35.36300493, 149.16517528, 50)
        # self.push_wps()
        
        # self.land()
        self.change_mode(MODE_RTL)
        self.disconnect()




if __name__ == "__main__":
    v = MyDroneHandler()
    v.connect("node1", rate=10)

