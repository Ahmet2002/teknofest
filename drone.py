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
        (success, integer, real) = self.set_param("WPNAV_SPEED", value_real=200.0)
        print(str(success), str(integer), str(real))
        self.arm(True)
        self.takeoff(1.0)

        self.move_global(x=5.0)
        # while True:
        #     self.set_vel_global(0.1)
        #     self.rate.sleep()
        # self.move_local(x=0.3)
        # self.move_local(y=0.3)
        # while True:
        #     print(f"yaw = {self.yaw * 180 /math.pi}")
        #     print("range for angle 0" + str(self.range_for_angle(0.0)))
        #     time.sleep(1)
        # self.move_global(yaw=270.0)
        # time.sleep(2)
        # self.align()

        # sentence = input("Please enter the sentence to be painted on the wall\n")
        # self.get_mission(sentence)
        # self.run_mission()
        # while not rospy.is_shutdown():
        #     self.print_ranges()
        #     self.rate.sleep()
        self.land()
        self.disconnect()




if __name__ == "__main__":
    v = MyDroneHandler()
    v.enable_topics_for_read()
    v.connect("node1", rate=10)

