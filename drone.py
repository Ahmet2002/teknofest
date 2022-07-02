import threading
import time
from mavros_python_examples.droneHandler import *


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
        
        print("arm:", self.armed, "mode:", self.mode)
        print(str(self.change_mode(MODE_GUIDED)))
        (success, integer, real) = self.set_param("WPNAV_SPEED", value_real=50.0)
        print(str(success), str(integer), str(real))
        self.arm(True)
        self.takeoff(4.0)

        self.move_global(yaw=180.0)
        # time.sleep(2)
        # self.initial_align()

        sentence = input("Please enter the sentence to be painted on the wall\n")
        self.get_mission(sentence)
        self.run_mission()
        (success, integer, real) = self.set_param("WPNAV_SPEED", value_real=1000.0)
        print(str(success), str(integer), str(real))
        self.change_mode(MODE_RTL)
        self.disconnect()




if __name__ == "__main__":
    v = MyDroneHandler()
    v.enable_topics_for_read()
    v.connect("node1", rate=10)

