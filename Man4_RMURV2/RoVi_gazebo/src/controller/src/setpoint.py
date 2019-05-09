#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float64
from math import radians, pow
import sys


def publish():
    rospy.init_node('setpoint', anonymous=True)
    setpoint_publisher = rospy.Publisher("/pid_controllers/z_controller/setpoint", Float64, queue_size=10)
    yawsetpoint_publisher = rospy.Publisher("/pid_controllers/yaw_controller/setpoint", Float64, queue_size=10)
    rate = rospy.Rate(0.1)
    value = Float64()
    value.data = 1
    while not rospy.is_shutdown():
        value.data = 30*11.4
        setpoint_publisher.publish(value)
        setpoint_publisher.publish(value)
        value.data = radians(180)
        yawsetpoint_publisher.publish(value)
        yawsetpoint_publisher.publish(value)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
