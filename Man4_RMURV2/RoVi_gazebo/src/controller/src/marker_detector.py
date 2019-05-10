#!/usr/bin/env python
import cv2 as cv
import roslib
import rospy
import sys
from sensor_msgs.msg import Image, Imu
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
        self.altitude_setpoint_publisher = rospy.Publisher("/pid_controllers/z_controller/setpoint", Float64, queue_size=10)
        self.IMU_sub = rospy.Subscriber("hummingbird/imu", Imu, self.angles)
        self.odom_sub = rospy.Subscriber("/hummingbird/odometry_sensor1/pose", Pose, self.pose)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("analyzed_image", Image, queue_size=10)
        self.dq = deque([0,0,0,0,0,0,0,0,0,0])
        self.angle = Float64()
        self.value = Float64()
        self.marker_xy = [0,0]
        self.RPY = [0,0,0]
        self.position = [0,0,0]

    def pose(self, pose):
        orientation_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        (R, P, Y) = euler_from_quaternion(orientation_list)
        self.RPY = [R,P,Y]
        #print(self.RPY)
        self.position = [pose.position.x, pose.position.y, pose.position.z]

    def angles(self, data):
        pass

    def marker_local_coordinates(self, img_midpoint, marker_xy, distance, angle):
        print 'marker is at [x: %.3f y: %.3f] ' % (self.position[0] + ((distance*cos(angle+self.RPY[2]))/12.3), self.position[1] +
                                                   ((distance*sin(angle+self.RPY[2]))/12.3))

        print 'own angle: %.3f' % (degrees(self.RPY[2]))
        print 'Angle %.3f' % (degrees(angle))
        print 'Angle minus own angle: %.3f' % (degrees(angle+self.RPY[2]))


    def unit_vector(self, vector):
        if np.linalg.norm(vector) == 0:
            return 0
        return vector / np.linalg.norm(vector)

    def approach(self, dist):
        if dist > 100:
            self.pitchsetpoint_publisher.publish(dist)
        else:
            self.pitchsetpoint_publisher.publish(0)

    def angle_between(self, v1, v2):
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def euclidean_dist(self, p1, p2):
        return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            pass
            #print(e)

        self.value.data = 40 * 11.4
        self.altitude_setpoint_publisher.publish(self.value)
        self.altitude_setpoint_publisher.publish(self.value)

        midpoint_y = (cv_image.shape[0])/2
        midpoint_x = (cv_image.shape[1])/2

        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        tracker = MarkerTracker.MarkerTracker(4, 50, 1)
        tracker.locate_marker(gray)

        if float(tracker.quality) > 0.8:
            self.marker_xy[0] = tracker.pose.x
            self.marker_xy[1] = tracker.pose.y
            dist = self.euclidean_dist((midpoint_x, midpoint_y), (tracker.pose.x, tracker.pose.y))

            cv.circle(cv_image, (tracker.pose.x, tracker.pose.y), 10, (255, 0, 0), thickness=-1)
            v1 = (midpoint_x-tracker.pose.x, midpoint_y-tracker.pose.y)
            v2 = (0, midpoint_y-tracker.pose.y)
            print"V2 is: %3.f" % (v1[0])

            if v1[0] > 0:
                self.angle = self.angle_between(v1,v2)
            else:
                self.angle = -self.angle_between(v1,v2)
            if abs(degrees(self.angle)) > 1:
                self.yawsetpoint_publisher.publish(self.angle)
            else:
                if self.position[2] > 39:
                    self.approach(dist)
                    self.marker_local_coordinates((midpoint_x,midpoint_y), (tracker.pose.x, tracker.pose.y), dist, self.angle)


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