#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float64
import sys


def publish():
    rospy.init_node('setpoint', anonymous=True)
    setpoint_publisher = rospy.Publisher("setpoint", Float64, queue_size=10)
    rate = rospy.Rate(0.1)
    value = Float64()
    value.data = 10
    while not rospy.is_shutdown():
        if value.data == 10:
            value.data = 20
        else:
            value.data = 10
            setpoint_publisher.publish(value)
            setpoint_publisher.publish(value)
            rate.sleep()


if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
