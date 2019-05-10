#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped, Pose
from std_msgs.msg import Float64
from mav_msgs.msg import RollPitchYawrateThrust
import sensor_msgs.msg
import sys
from tf.transformations import euler_from_quaternion


class translate():
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=True)
        self.PIDz_state = rospy.Publisher("pid_controllers/z_controller/state", Float64, queue_size=10)
        self.PIDz_Sub = rospy.Subscriber("pid_controllers/z_controller/control_effort", Float64, self.pidaltitude)
        self.PIDyaw_state = rospy.Publisher("pid_controllers/yaw_controller/state", Float64, queue_size=10)
        self.PIDyaw_Sub = rospy.Subscriber("pid_controllers/yaw_controller/control_effort", Float64, self.pidyaw)
        self.PIDpitch_state = rospy.Publisher("pid_controllers/pitch_controller/state", Float64, queue_size=10)
        self.PIDpitch_Sub = rospy.Subscriber("pid_controllers/pitch_controller/control_effort", Float64, self.pidpitch)
        self.PIDroll_state = rospy.Publisher("pid_controllers/roll_controller/state", Float64, queue_size=10)
        self.PIDroll_Sub = rospy.Subscriber("pid_controllers/roll_controller/control_effort", Float64, self.pidroll)
        self.altimeter_sub = rospy.Subscriber("hummingbird/air_pressure", sensor_msgs.msg.FluidPressure, self.height)
        self.odom_sub = rospy.Subscriber("/hummingbird/odometry_sensor1/pose", Pose, self.pose)
        self.humm_rotor_pub = rospy.Publisher("/hummingbird/command/roll_pitch_yawrate_thrust", RollPitchYawrateThrust,
                                              queue_size=10)
        self.yaw_control_effort = Float64()
        self.pitch_control_effort = Float64()
        self.roll_control_effort = Float64()
        self.pose = Pose()

    def pose(self, pose):
        self.pose.position = pose.position
        self.pose.orientation = pose.orientation

    # get roll control effort from PID controller
    def pidroll(self, data):
        self.roll_control_effort.data = data.data

    # get pitch control effort from PID controller
    def pidpitch(self, data):
        self.pitch_control_effort.data = data.data

    # get yaw control effort from PID controller
    def pidyaw(self, data):
        self.yaw_control_effort.data = data.data

    # getting height of the drone.
    def height(self, data):
        pressure = sensor_msgs.msg.FluidPressure()
        pressure.fluid_pressure = data.fluid_pressure
        self.PIDz_state.publish(-pressure.fluid_pressure + 95460)

    # Not used right now, just tells us our rotation and such.
    def angles(self, data):
        orientation_and_pose = sensor_msgs.msg.Imu()
        orientation_and_pose.orientation = data.orientation
        orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (R, P, Y) = euler_from_quaternion(orientation_list)

    # As long as we are in the air, send motor commands
    def pidaltitude(self, data):
        # We wish that our drones current position error is 0, the setpoint is the distance to the object / position we-
        # want to go to therefore we publish 0 to the PID state.
        self.PIDyaw_state.publish(0)
        self.PIDpitch_state.publish(0)
        self.PIDroll_state.publish(0)
        rotor_speed = RollPitchYawrateThrust()
        rotor_speed.roll = self.roll_control_effort.data
        rotor_speed.yaw_rate = self.yaw_control_effort.data
        rotor_speed.pitch = self.pitch_control_effort.data
        rotor_speed.thrust.z = float(data.data / 10.) + 7.31
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
