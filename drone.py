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
        self.arm(True)
        self.takeoff(3.0)
        
        while self.z < 2.94 :
            print(str(self.z))
            self.rate.sleep()

        sentence = input("Please enter the sentence to be painted on the wall\n")
        self.get_mission(sentence)
        self.run_mission()
        # while not rospy.is_shutdown():
        #     self.print_ranges()
        #     self.rate.sleep()
        self.land()
        while self.z > 0.03:
            print(str(self.z))
            self.rate.sleep()
        self.disconnect()




if __name__ == "__main__":
    v = MyDroneHandler()
    v.enable_topics_for_read()
    v.connect("node1", rate=10)
