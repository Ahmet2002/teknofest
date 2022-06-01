import threading
import time
from mavros_python_examples.droneHandler import *


class MyRoverHandler(DroneHandler):
    def __init__(self):
        super().__init__()

        self.user_thread = threading.Timer(0, self.user)
        self.user_thread.daemon = True
        self.user_thread.start()

    def user(self):
        while not self.connected:
            rospy.loginfo("Waiting to be connected.")
            time.sleep(1)
        print("arm:", self.armed, "mode:", self.mode)
        print(str(self.change_mode(MODE_GUIDED)))
        self.arm(True)
        self.takeoff(4)
        
        while self.z < 3.96 :
            print(str(self.z))
            self.rate.sleep()
        self.move2target(-2.0, -1.0, 1.5)
        self.move2target(2.0, 1.0, 0.5)
        self.land()
        while self.z > 0.03:
            print(str(self.z))
            time.sleep(1)
        self.disconnect()




if __name__ == "__main__":
    v = MyRoverHandler()
    v.enable_topics_for_read()
    v.connect("node1", rate=10)

