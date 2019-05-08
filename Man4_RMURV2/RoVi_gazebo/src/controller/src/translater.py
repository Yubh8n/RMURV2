#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Vector3, PointStamped
from std_msgs.msg import Float64
from mav_msgs.msg import RollPitchYawrateThrust
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
        #self.Rocket_sub = rospy.Subscriber("location", Point, self.roccallback)
        self.rotors_sub = rospy.Subscriber("/hummingbird/ground_truth/position", PointStamped, self.humming_sub)
        #self.rotors_pub = rospy.Publisher("altitude", Float64, queue_size=10)
        self.PID_Pub = rospy.Publisher("state", Float64, queue_size=10)
        self.PID_Sub = rospy.Subscriber("control_effort", Float64, self.pidcallback)
        #self.Rocket_Pub = rospy.Publisher("thrust", Vector3, queue_size=10)
        self.humm_rotor_pub = rospy.Publisher("/hummingbird/command/roll_pitch_yawrate_thrust", RollPitchYawrateThrust, queue_size=10)

    def humming_sub(self, data):
        Y_pos = Float64()
        Y_pos.data = data.point.z
        self.PID_Pub.publish(Y_pos)

    def roccallback(self, data):
        Y_pos = Point()
        Y_pos.y = data.y
        correct_pos = Float64()
        correct_pos.data = Y_pos.y
        self.PID_Pub.publish(correct_pos.data)

    def pidcallback(self, data):

        rotor_speed = RollPitchYawrateThrust()
        thrust = Vector3()
        thrust.z = data.data + 7.31 #thrust in z axis in newton, 7.31 is approx the equilibrium
        thrust.x = 0.0
        thrust.y = 0.0
        rotor_speed.thrust = thrust
        self.humm_rotor_pub.publish(rotor_speed)


def main(args):
    trans = translate()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    print("Launching!")
    main(sys.argv)
