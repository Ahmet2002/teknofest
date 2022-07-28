import threading
import time
from utilities.droneHandler3 import *


class MyDroneHandler(DroneHandler3):
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
        self.move_global(yaw=0.0)
        # self.move_local(y=-7.0)
        
        self.change_mode(MODE_RTL)
        self.disconnect()




if __name__ == "__main__":
    v = MyDroneHandler()
    v.connect("node1", rate=10)

