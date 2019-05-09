#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Vector3, PointStamped
from std_msgs.msg import Float64
from mav_msgs.msg import RollPitchYawrateThrust
import sensor_msgs.msg
import sys
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees

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
        self.PIDz_Pub = rospy.Publisher("pid_controllers/z_controller/state", Float64, queue_size=10)
        self.PIDz_Sub = rospy.Subscriber("pid_controllers/z_controller/control_effort", Float64, self.pidcallback)
        self.PIDyaw_Pub = rospy.Publisher("pid_controllers/yaw_controller/state", Float64, queue_size=10)
        self.PIDyaw_Sub = rospy.Subscriber("pid_controllers/yaw_controller/control_effort", Float64, self.pidyaw)
        self.IMU_sub = rospy.Subscriber("hummingbird/imu", sensor_msgs.msg.Imu, self.angles)
        self.altimeter_sub = rospy.Subscriber("hummingbird/air_pressure", sensor_msgs.msg.FluidPressure, self.height)
        #self.Rocket_Pub = rospy.Publisher("thrust", Vector3, queue_size=10)
        self.humm_rotor_pub = rospy.Publisher("/hummingbird/command/roll_pitch_yawrate_thrust", RollPitchYawrateThrust, queue_size=10)
        self.yaw_control_effort = Float64()

    def height(self, data):
        pressure = sensor_msgs.msg.FluidPressure()
        pressure.fluid_pressure = data.fluid_pressure
        self.PIDz_Pub.publish(-pressure.fluid_pressure + 95460)
        #print(-pressure.fluid_pressure + 95460)
    def angles(self, data):
        orientation_and_pose = sensor_msgs.msg.Imu()
        orientation_and_pose.orientation = data.orientation
        orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (R, P, Y) = euler_from_quaternion(orientation_list)
        #print("Roll: ", degrees(R)," Pitch: ",degrees(P)," Yaw: ",degrees(Y))
        self.PIDyaw_Pub.publish(0)
    def humming_sub(self, data):
        Z_pos = Float64()
        Z_pos.data = data.point.z - 95460
        #self.PIDz_Pub.publish(Z_pos)
    def pidyaw(self,data):
        self.yaw_control_effort.data = data.data
    def pidcallback(self, data):
        rotor_speed = RollPitchYawrateThrust()
        rotor_speed.yaw_rate = self.yaw_control_effort.data
        rotor_speed.thrust.z = float(data.data/10.) + 7.31
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
