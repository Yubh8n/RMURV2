#!/usr/bin/python
import numpy as np
import time
import rospy
from std_msgs.msg      import Int32
from object_publisher.msg import example_objects

x_1 = 0.0
x_2 = 0.0
y_1 = 0.0
y_2 = 0.0

R1 = 40
R2 = 100
pub_objects = rospy.Publisher('/object_list', example_objects, queue_size=10)

def time_event_handle(event):
    msg = example_objects()
    t = rospy.get_time()
    msg.header.stamp = rospy.Time.now()
    x_1 = R1*np.sin(t)+2*R1
    y_1 = R1*np.cos(t)+1.8*R1
    msg.IDs.append(1)
    msg.x.append(x_1)
    msg.y.append(y_1)
    x_2 = R2*np.cos(t)+2*R2
    y_2 = R2*np.sin(t)+1.5*R2
    msg.IDs.append(2)
    msg.x.append(x_2)
    msg.y.append(y_2)
    pub_objects.publish(msg)


if __name__ == '__main__':
    rospy.init_node('example_object_publisher')
    rospy.Timer(rospy.Duration(0.1), time_event_handle)
    rospy.spin()
