#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float64
import sys

#From   rocket location
#To     Translater
#To     PID state


#From   PID control
#To     Translater
#To     rocket thrust



class translate():
    def __init__(self):
        rospy.init_node('translater', anonymous=True)
        self.Rocket_sub = rospy.Subscriber("location", Point, self.roccallback)
        self.PID_Pub = rospy.Publisher("state", Float64, queue_size=10)
        self.PID_Sub = rospy.Subscriber("control_effort", Float64, self.pidcallback)
        self.Rocket_Pub = rospy.Publisher("thrust", Vector3, queue_size=10)

    def roccallback(self, data):
        Y_pos = Point()
        Y_pos.y = data.y
        correct_pos = Float64()
        correct_pos.data = Y_pos.y
        self.PID_Pub.publish(correct_pos.data)

    def pidcallback(self, data):
        thrust = Float64()
        thrust.data = data.data
        Y_thrust = Vector3()
        Y_thrust.y = data.data
        self.Rocket_Pub.publish(Y_thrust)


def main(args):
    trans = translate()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    print("Launching!")
    main(sys.argv)