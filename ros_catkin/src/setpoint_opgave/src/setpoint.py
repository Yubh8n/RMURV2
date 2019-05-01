#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float64
import sys



        
def publish():
    rospy.init_node('setpoint', anonymous=True)
    setpoint_publisher = rospy.Publisher("setpoint", Float64, queue_size=10)
    rate = rospy.Rate(0.2)
    value = Float64()
    value.data = 100
    while not rospy.is_shutdown():
        setpoint_publisher.publish(value)
        #value.data = -value.data
        setpoint_publisher.publish(value)
        rate.sleep()
	value.data += 10
	if value.data > 500:
		value.data = 0


if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
