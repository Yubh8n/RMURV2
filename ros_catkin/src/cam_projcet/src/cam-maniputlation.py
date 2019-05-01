#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
roslib.load_manifest('cam_projcet')
import rospkg

class receiver:
    def __init__(self):
        rospy.init_node('image_shower', anonymous=True)
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.callback)  # Image is not the image, but image from sensor_msgs.msgs
        self.bridge = CvBridge()
        self.current_fps = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        Light_image = self.enlighten(cv_image)
        cv2.imshow("ros image stream", cv_image)
        cv2.waitKey(1)

#Function needs some doing..
    def enlighten(self, image):

        b, g, r = cv2.split(image)
        b += 50
        g += 50
        r += 50
        final_img = cv2.merge((b, g, r))
        return final_img
        pass

def main(args):
    ic = receiver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    print("Launching!")
    main(sys.argv)
