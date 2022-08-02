from utilities.utils import *
import rospy
from geopy import distance

class MixinPublishing:

    def set_vel_global(self, xprime=0.0, yprime=0.0, zprime=0.0, yaw_vel=0.0):
        data = geometry_msgs.msg.Twist()
        data.linear.x = xprime
        data.linear.y = yprime
        data.linear.z = zprime
        data.angular.z = yaw_vel
        data.angular.y = 0.0
        data.angular.x = 0.0
        self.pub_vel_global.publish(data)

    def set_vel_local(self, xprime=0.0, yprime=0.0, zprime=0.0, yaw_vel=0.0):
        (xprime, yprime) = self.transform(xprime, yprime)
        self.set_vel_global(xprime, yprime, zprime, yaw_vel)

    def move_global_rel(self, x=None, y=None , z=None, yaw=None):
        if x == None:
            x = self.x
        if y == None:
            y = self.y
        if z == None:
            z = self.z
        if yaw == None:
            yaw = self.yaw
        else:
            yaw = angle2radian(yaw)
        data = geometry_msgs.msg.PoseStamped()
        data.header.stamp = rospy.Time.now()
        data.pose.position.x = x
        data.pose.position.y = y
        data.pose.position.z = z
        (data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,
        data.pose.orientation.w) = get_quaternion_from_euler(self.roll, self.pitch, yaw)
        self.pub_pose_local.publish(data)

        while not rospy.is_shutdown():
            self.print_pose()
            if self.is_target_reached(x, y, z, yaw):
                break
            self.rate.sleep()

    def move_local(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        (x, y) = self.transform(x, y)
        x += self.x
        y += self.y
        z += self.z
        self.move_global_rel(x, y, z, 180 / math.pi * self.yaw + yaw)

    def is_target_reached(self, x, y, z, yaw=None, check_yaw=True, tolerance_lin=0.2, tolerance_ang=0.2, is_global=False):
        if not is_global:
            dx = self.x - x
            dy = self.y - y
            dz = self.z - z
            distance = math.sqrt((math.pow(dx, 2) + math.pow(dy, 2) + math.pow(dz, 2)))
        
        else:
            check_yaw = False
            distance = self.iki_nokta_arasi_uzaklik_hesaplama_3d((x, y, z + self.home[2]), (self.latitude, self.longitude, self.altitude))
        print("distance : ", distance)

        if (distance <= tolerance_lin):
            if check_yaw:
                dyaw = self.yaw - yaw
                dyaw = abs(math.atan2(math.sin(dyaw), math.cos(dyaw)))
                if dyaw > tolerance_ang:
                    return False
            return True
        return False

    def transform(self, x:float, y:float, angle=None): # not finished
        if not angle:
            angle= self.yaw - math.pi / 2
        tmp_x = x
        x = math.cos(angle) * tmp_x - math.sin(angle) * y
        y = math.cos(angle) * y + math.sin(angle) * tmp_x
        return x, y

    def give_global_loc(self, lat:float, lon:float, alt:float):
        data = mavros_msgs.msg.GlobalPositionTarget()
        data.latitude = lat
        data.longitude = lon
        data.altitude = alt
        data.type_mask = data.IGNORE_YAW + data.IGNORE_AFX + data.IGNORE_AFY + data.IGNORE_AFZ + data.IGNORE_VX + data.IGNORE_VY + data.IGNORE_VZ + data.IGNORE_YAW_RATE
        data.coordinate_frame = data.FRAME_GLOBAL_REL_ALT
        data.header.stamp = rospy.Time.now()
        self.pub_pose_global.publish(data)
        while not rospy.is_shutdown():
            self.print_pose_global()
            if self.is_target_reached(lat, lon, alt, is_global=True):
                break
            self.rate.sleep()

    @staticmethod
    def iki_nokta_arasi_uzaklik_hesaplama_2d(coord_1,coord_2):
        distance_2d = distance.distance(coord_1[:2], coord_2[:2]).m
        #print("2D - " +str(distance_2d))
        return distance_2d

    def iki_nokta_arasi_uzaklik_hesaplama_3d(self, coord_1,coord_2):
        distance_3d = np.sqrt(self.iki_nokta_arasi_uzaklik_hesaplama_2d(coord_1,coord_2)**2 + (coord_1[2] - coord_2[2])**2)
        #print("3D - "+str(distance_3d))
        return distance_3d

class Waypoints():
	def __init__(self):
		self.x_global = self.latitude
		self.y_global = self.longitude
		self.z_global = self.altitude
		self.yawRate = (180 / math.pi * self.yaw) + 90
		self.fontSize = 0.0

	def wpCreater(self, fontSize):
		for letter in self.wall.sentence:
			if letter == "T":
				wp_T = []
				current_wp = [self.x_global,self.y_global,self.z_global]
				wp_T.append(current_wp)
				wp1 = newLoactionCal(current_wp, fontSize, self.yawRate)
				wp_T.append(tuple(wp1))
				wp2 = newLoactionCal(current_wp, (-fontSize/2), self.yawRate)
				wp_T.append(tuple(wp2))
				wp3 = wp2.copy()
