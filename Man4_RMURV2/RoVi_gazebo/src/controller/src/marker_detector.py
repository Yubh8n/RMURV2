#!/usr/bin/env python
import cv2 as cv
import roslib
import rospy
import sys
from sensor_msgs.msg import Image, Imu, FluidPressure
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
import MarkerTracker
from collections import deque
import numpy as np
from math import sqrt, cos, sin, degrees
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class reciever:
    def __init__(self):
        rospy.init_node('image_shower', anonymous=True)
        self.image_sub = rospy.Subscriber("hummingbird/camera_/image_raw", Image, self.callback)  # Image is not the image, but image from sensor_msgs.msgs
        self.yawsetpoint_publisher = rospy.Publisher("/pid_controllers/yaw_controller/setpoint", Float64, queue_size=10)
        self.pitchsetpoint_publisher= rospy.Publisher("/pid_controllers/pitch_controller/setpoint", Float64, queue_size=10)
        self.rollsetpoint_publisher= rospy.Publisher("/pid_controllers/roll_controller/setpoint", Float64, queue_size=10)
        self.altitude_setpoint_publisher = rospy.Publisher("/pid_controllers/z_controller/setpoint", Float64, queue_size=10)
        self.IMU_sub = rospy.Subscriber("hummingbird/imu", Imu, self.angles)
        self.altimeter_sub = rospy.Subscriber("hummingbird/air_pressure", FluidPressure, self.height)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("analyzed_image", Image, queue_size=10)
        self.angle = Float64()
        self.value = Float64()
        self.value.data = 40 * 10.999346831
        self.marker_xy = [0,0]
        self.RPY = [0,0,0]
        self.marker_local_pos = [0,0]
        self.altitude = 0


    def height(self, data):
        pressure = FluidPressure()
        pressure.fluid_pressure = (95459.97 - data.fluid_pressure)/10.99934
        self.altitude = pressure.fluid_pressure
        #print self.altitude


    def pose(self, pose):
        orientation_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        (R, P, Y) = euler_from_quaternion(orientation_list)
        self.RPY = [R,P,Y]
        #print(self.RPY)
        self.position = [pose.position.x, pose.position.y, pose.position.z]

    def angles(self, data):
        orientation_and_pose = Imu()
        orientation_and_pose.orientation = data.orientation
        orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (R, P, Y) = euler_from_quaternion(orientation_list)
        self.RPY[0] = degrees(R)
        self.RPY[1] = degrees(P)
        self.RPY[2] = degrees(Y)

    def unit_vector(self, vector):
        if np.linalg.norm(vector) == 0:
            return 0
        return vector / np.linalg.norm(vector)

    def approach(self, dist):
        self.pitchsetpoint_publisher.publish(dist)

    def angle_between(self, v1, v2):
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def euclidean_dist(self, p1, p2):
        return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def decent(self):
        print "Setpoint Lower bound: " ,self.value.data, "Setpoint upper bound: ", self.value.data
        print "Current altitude: ", self.altitude
        if self.value.data/10 - 1 < self.altitude < self.value.data/10 + 1:
            print "decending!"
            self.value.data = self.value.data - 50


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            pass
            #print(e)

        self.altitude_setpoint_publisher.publish(self.value)

        print self.RPY

        midpoint_y = (cv_image.shape[0])/2
        midpoint_x = (cv_image.shape[1])/2

        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        tracker = MarkerTracker.MarkerTracker(4, 50, 1)
        tracker.locate_marker(gray)

        if float(tracker.quality) > 0.8:
            self.marker_xy[0] = tracker.pose.x
            self.marker_xy[1] = tracker.pose.y

            dist_img = self.euclidean_dist((midpoint_x, midpoint_y), (tracker.pose.x, tracker.pose.y))
            cv.circle(cv_image, (tracker.pose.x, tracker.pose.y), 1, (255, 0, 0), thickness=-1)
            v1 = (midpoint_x-tracker.pose.x, midpoint_y-tracker.pose.y)
            v2 = (0, midpoint_y-tracker.pose.y)

            if v1[0] > 0:
                self.angle = self.angle_between(v1,v2)
            else:
                self.angle = -self.angle_between(v1,v2)
            self.yawsetpoint_publisher.publish(self.angle)
            self.rollsetpoint_publisher.publish(self.angle)


            #print dist_img
            #print degrees(self.angle)
            if degrees(abs(self.angle)) < 10:
                self.approach(dist_img)
                if dist_img < 100:
                    #print "decending!"
                    self.decent()

        cv.namedWindow("drone image", cv.WINDOW_NORMAL)
        cv.imshow("drone image", cv_image)
        cv.waitKey(1)

def main(args):
    ic = reciever()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()


if __name__ == '__main__':
    print("Launching!")
    main(sys.argv)