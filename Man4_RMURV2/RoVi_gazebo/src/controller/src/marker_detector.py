#!/usr/bin/env python
import cv2 as cv
import roslib
import rospy
import sys
from collections import deque
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
        self.ground_truth_sub = rospy.Subscriber("hummingbird/ground_truth/pose", Pose, self.pose)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("analyzed_image", Image, queue_size=10)
        self.angle = Float64()
        self.value = Float64()
        self.value.data = 40 * 10.999346831
        self.marker_xy = [0,0]
        self.RPY = [0,0,0]
        self.marker_local_pos = [0,0]
        self.altitude = 0
        self.state = 0
        self.ground_truth_altitude = 0
        self.pose = 0
        self.dq_angle = deque([0,0,0,0,0,0,0,0,0,0])

    # Get Marker pose in image
    def marker_pos(self, x, y):
        print "marker is at: ", y - sin(self.RPY[1])*(self.altitude-5)

    # get ground truth
    def pose(self, data):
        pose = Pose()
        pose.position = data.position
        #print pose.position
        self.ground_truth_altitude = pose.position.z
        self.pose = pose.position

    # get height data readings from pressure sensor
    def height(self, data):
        pressure = FluidPressure()
        pressure.fluid_pressure = (95460 - data.fluid_pressure)/11.4
        self.altitude = pressure.fluid_pressure

    # get the angles of the drone from IMU data.
    def angles(self, data):
        orientation_and_pose = Imu()
        orientation_and_pose.orientation = data.orientation
        orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (R, P, Y) = euler_from_quaternion(orientation_list)
        self.RPY[0] = degrees(R)
        self.RPY[1] = degrees(P)
        self.RPY[2] = degrees(Y)

    # used to find angles between two vectors. (For yawing towards the marker)
    def unit_vector(self, vector):
        if np.linalg.norm(vector) == 0:
            return 0
        return vector / np.linalg.norm(vector)

    def center(self, dist):
        print self.RPY[1]

    # finds the angle between two vectors.
    def angle_between(self, v1, v2):
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    # calculates the euclidean distance between two points.
    def euclidean_dist(self, p1, p2):
        return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    # Lowers the drone by 5 meters.
    def decent(self):
        #print "Setpoint Lower bound: " ,self.value.data, "Setpoint upper bound: ", self.value.data
        #print "Current altitude: ", self.altitude
        if self.value.data/10 - 1 < self.altitude < self.value.data/10 + 1:
            print "decending!"
            self.value.data = self.value.data - 50

    # Align the front of the drone to point towards the marker
    def align(self, angle):
        self.dq_angle.append(angle)
        self.yawsetpoint_publisher.publish(angle)
        if degrees(np.mean(self.dq_angle)) == 0:
            self.state = 1
        else:
            print degrees(np.mean(self.dq_angle))
        self.dq_angle.popleft()

    # publish the distance to the marker to approach.
    def approach(self, dist):
        if abs(dist) < -5:
            # self.pitchsetpoint_publisher.publish(0)
            # self.state = 2
            pass
        else:
            self.pitchsetpoint_publisher.publish(dist)
            print dist

    # Main routine (from an image, do some image magic)
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            pass
            #print(e)

        # send the drone to the self.value height.
        self.altitude_setpoint_publisher.publish(self.value)

        # Print the roll pitch and yaw of the drone.
        #print self.RPY
        #print self.altitude - self.ground_truth_altitude

        # Find the midpoint of the image (used to find the distance to the marker)
        midpoint_y = (cv_image.shape[0])/2
        midpoint_x = (cv_image.shape[1])/2

        # Locate the marker in the image
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        tracker = MarkerTracker.MarkerTracker(4, 50, 1)
        tracker.locate_marker(gray)

        # if the marker is found, and the tracker quality is of 80% certainty:
        if float(tracker.quality) > 0.8:
            # Find the markers x.y in the image
            self.marker_xy[0] = tracker.pose.x# - (sin(degrees(self.RPY[0]))*self.altitude)
            self.marker_xy[1] = tracker.pose.y# - (sin(degrees(self.RPY[1]))*self.altitude)
            self.marker_pos(tracker.pose.x, tracker.pose.y)

            # Draw a circle at the tracker.
            dist_img = (self.euclidean_dist((midpoint_x, midpoint_y), (self.marker_xy[0], self.marker_xy[1])))/(self.altitude-5)
            print "marker x: ", self.marker_xy[1], " x midpoint of image: ", midpoint_y
            if self.marker_xy[1] > midpoint_y:
                dist_img = - dist_img
            dist_ground_truth = (self.euclidean_dist((self.pose.x, self.pose.y), (10.7, 14.33)))
            #print "The distance to the image: ", dist_img, " True distance is: ", dist_ground_truth

            cv.circle(cv_image, (tracker.pose.x, tracker.pose.y), 1, (255, 0, 0), thickness=-1)

            # Determine two vectors, one pointing towards the marker tracker
            v1 = (midpoint_x-tracker.pose.x, midpoint_y-tracker.pose.y)

            # Points straight forward.
            v2 = (0, midpoint_y-tracker.pose.y)

            # If the object is to the left, the angle is positive (for yawing)
            if v1[0] > 0:
                self.angle = self.angle_between(v1, v2)

            # if the object is to the right, the angle is negative (not really, but this works for yawing)
            else:
                self.angle = -self.angle_between(v1, v2)

            if self.state == 0:
                # publish the angle to the yaw controller, to yaw until the marker is in front of the drone
                self.align(self.angle)
            if self.state == 1:
                self.approach(dist_img)
            if self.state == 2:
                self.center(dist_img)
                pass
            if self.state == 3:
                #self.decent
                pass
            # Roll to decrease the angle "Y" error in the image.
            # self.rollsetpoint_publisher.publish(self.angle)

        # Show the image from the drone.
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