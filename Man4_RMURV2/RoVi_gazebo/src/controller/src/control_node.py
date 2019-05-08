#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32




class altitude_control:
    def __init__(self):
        rospy.init_node("controller_node", anonymous=True)
        self.altitide_sub = rospy.Subscriber("/hummingbird/ground_truth/position", PointStamped, self.altitude_callback)
        self.PID_Pub = rospy.Publisher("state", Float32, queue_size=10)
        self.PID_Sub = rospy.Subscriber("control_effort", Float32, self.pidcallback)
        self.alti_float_pub = rospy.Publisher("altitude_float", Float32, queue_size=10)
        rospy.spin()

    def pidcallback(self, data):
        pass

    def altitude_callback(self, data):
        pub_data = Float32()
        pub_data.data = data.point.z
        self.alti_float_pub.publish(pub_data)


if __name__ == "__main__":
    print("Running control_node")
    ac = altitude_control()