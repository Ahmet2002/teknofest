import rospy
from mavros_python_examples.service import Service


class RosHandler:
    def __init__(self):
        self.frequency = 10
        self.connected = False
        

    def connect(self, node: str, rate: int):
        rospy.init_node(node, anonymous=True)
        self.rate = rospy.Rate(self.frequency)
        self.connected = True
        rospy.loginfo("Rospy is up ...")
        rospy.spin()

    def disconnect(self):
        if self.connected:
            rospy.loginfo("shutting down rospy ...")
            rospy.signal_shutdown("disconnect")
            self.connected = False

    @staticmethod
    def topic_publisher(topic: Service):
        pub = rospy.Publisher(topic.get_name(), topic.get_type(), queue_size=10)
        pub.publish(topic.get_data())
        print("Publishing")

    def topic_subscriber(topic: Service):
        rospy.Subscriber(topic.get_name(), topic.get_type(), topic.set_data)

    def service_caller(self, service: Service, timeout=30):
        try:
            srv = service.get_name()
            typ = service.get_type()
            data = service.get_data()

            rospy.loginfo("waiting for ROS service:" + srv)
            rospy.wait_for_service(srv, timeout=timeout)
            rospy.loginfo("ROS service is up:" + srv)
            call_srv = rospy.ServiceProxy(srv, typ)
            return call_srv(data)
        except rospy.ROSException as e:
            print("ROS ERROR:", e)
        except rospy.ROSInternalException as e:
            print("ROS ERROR:", e)
        except KeyError as e:
            print("ERROR:", e)
        return None

