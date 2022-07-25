import threading
import time
from mavros_python_examples.droneHandler3 import *


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

        self.move_global(yaw=160.0)

        sentence = input("Please enter the sentence to be painted on the wall\n")
        self.get_mission(sentence)
        self.run_mission()
        (success, integer, real) = self.set_param("WPNAV_SPEED", value_real=400.0)
        print(str(success), str(integer), str(real))
        self.change_mode(MODE_RTL)
        self.disconnect()




if __name__ == "__main__":
    v = MyDroneHandler()
    v.connect("node1", rate=10)

