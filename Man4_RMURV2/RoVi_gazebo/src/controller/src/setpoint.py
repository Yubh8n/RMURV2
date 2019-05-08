#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float64
import sys


def publish():
    rospy.init_node('setpoint', anonymous=True)
    setpoint_publisher = rospy.Publisher("setpoint", Float64, queue_size=10)
    rate = rospy.Rate(0.1)
    value = Float64()
    value.data = 1
    while not rospy.is_shutdown():
        if value.data == 1:
            value.data = 2
        else:
            value.data = 1
        #setpoint_publisher.publish(value)
        #setpoint_publisher.publish(value)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
