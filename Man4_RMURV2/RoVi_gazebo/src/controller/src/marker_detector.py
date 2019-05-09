#!/usr/bin/env python
import cv2 as cv
import roslib
import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import MarkerTracker
from collections import deque
import numpy as np
from math import sqrt
from std_msgs.msg import Float64

class reciever:
    def __init__(self):
        rospy.init_node('image_shower', anonymous=True)
        self.image_sub = rospy.Subscriber("hummingbird/camera_/image_raw", Image, self.callback)  # Image is not the image, but image from sensor_msgs.msgs
        self.yawsetpoint_publisher = rospy.Publisher("/pid_controllers/yaw_controller/setpoint", Float64, queue_size=10)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("analyzed_image", Image, queue_size=10)
        self.dq = deque([0,0,0,0,0,0,0,0,0,0])
        self.angle = Float64()

    def unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)

    def angle_between(self, v1, v2):
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def euclidean_dist(self, p1, p2):
        dist = sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        return dist

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        midpoint_y = (cv_image.shape[0])/2
        midpoint_x = (cv_image.shape[1])/2

        #print(height, width)
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        tracker = MarkerTracker.MarkerTracker(4, 50, 1)
        tracker.locate_marker(gray)
        self.dq.appendleft(tracker.quality)


        if float(np.mean(self.dq)) > 0.7:
            cv.circle(cv_image, (tracker.pose.x, tracker.pose.y), 10, (255, 0, 0), thickness=10)
            print("tracker in x: ", tracker.pose.x, " tracker in y: ", tracker.pose.y)
            print("Img height is: ", cv_image.shape[0], " Img width is: ", cv_image.shape[1])
            print(tracker.pose.x-midpoint_x, tracker.pose.y-midpoint_y)
            print(tracker.pose.x-midpoint_x, 0)
            v1 = (tracker.pose.x-midpoint_x, tracker.pose.y-midpoint_y)
            v2 = (0, tracker.pose.y-midpoint_y)
            print("angle in radians is: ",self.angle_between(v1,v2))
            self.angle = self.angle_between(v1,v2)
            if tracker.pose.x-midpoint_x > 0:
                self.yawsetpoint_publisher.publish(-self.angle)
            else:
                self.yawsetpoint_publisher.publish(self.angle)


        cv.namedWindow("test", cv.WINDOW_NORMAL)
        cv.imshow("test", cv_image)


        self.dq.pop()
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