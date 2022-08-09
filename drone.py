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

        self.takeoff(5.0)
        time.sleep(2.0)


        # self.duvara_bak(distance=2.0)
        # self.wall.sentence = input("Type the sentence.\n")
        # self.run_mission()
        self.run_mission_without_lidar(3.14, 1.5)

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

