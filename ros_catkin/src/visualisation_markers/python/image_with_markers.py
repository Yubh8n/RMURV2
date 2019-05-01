#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from object_publisher.msg import example_objects

class ImageDisplay:
    def __init__(self):
        plt.ion()
        rospy.init_node('image_displaywithmarkers')
        self.image_subscriber = rospy.Subscriber("image_raw", Image, self.get_frame)
        self.marker_subscriber = rospy.Subscriber('/object_list', example_objects, self.get_markers)
        rospy.Timer(rospy.Duration(0.2), self.time_event_handle)
        self.bridge = CvBridge()
        self.got_frame = False
        self.fame = Image()
        self.got_markers = False
        self.current_markers = example_objects()
        self.display_frame = True
        self.textoffset = 30
        rospy.spin()

    def time_event_handle(self, event):
        if(self.got_frame):
            imgplot = plt.imshow(self.frame)
            self.display_frame = True
        if(self.got_markers):
            ids = self.current_markers.IDs
            for i in range(0,len(ids)):
                id = ids[i]
                x_p = self.current_markers.x[i]
                y_p = self.current_markers.y[i]
                plt.plot(x_p, y_p, 'xk')
                plt.text(x_p, y_p+self.textoffset, str(id), fontsize=12) # 20 is a small offset value
        if(self.got_frame or self.got_markers):
            plt.draw()
            plt.pause(0.01)
            plt.clf()


    def get_markers(self, msg):
        self.current_markers = msg
        self.got_markers = True

    def get_frame(self, data):
        try:
            self.got_frame = True
            if(self.display_frame):
                self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
                self.display_frame = False #
        except CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    node = ImageDisplay()
