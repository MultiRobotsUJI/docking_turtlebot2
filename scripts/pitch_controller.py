#!/usr/bin/env python
import rospy
from pid import PID
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

import tf
import math

from PI_Controller import*

class YawController:

    def __init__(self):

        self.pub = rospy.Publisher('controller/pitch/effort', Float64MultiArray, queue_size=10)
        
        self.init = False

        self.startup_pitch = 0.0
        self.desired_val = 0

        self.step = 0.02
        self.prev_time = 0

        self.controller = PID()

        self.get_params()
        self.set_controller()
        
        self.sensor_sub = rospy.Subscriber(
            "mavros/imu/data", Imu, self.sensor_callback)
        self.reset_sub = rospy.Subscriber(
            "controllers/reset", Empty, self.reset_callback)
        self.desired_val_sub = rospy.Subscriber(
            "controller/pitch/desired", Float64, self.desired_val_callback)

    def set_controller(self):
        self.controller.set_gains(self.kp, self.ki, self.kd)
        self.controller.set_step(self.step)
        
    def desired_val_callback(self, msg):
        self.desired_val = msg.data
        if self.desired_val > 180:
            self.desired_val = self.desired_val - 360
        if self.desired_val <= -180:
            self.desired_val = self.desired_val + 360
    def get_params(self):
        self.kp = rospy.get_param('controller/pitch/kp', 0.0)
        self.ki = rospy.get_param('controller/pitch/ki', 0.0)
        self.kd = rospy.get_param('controller/pitch/kd', 0.0)
        print(f"kp = {self.kp}")

    def reset_callback(self, data):
        self.controller.reset_controller()
        self.init = True

    def sensor_callback(self,data):
        orientation = data.orientation
        angular_velocity = data.angular_velocity
        
        # update dt
        curr_time = rospy.Time.now().to_sec()
        dt = curr_time - self.prev_time
        self.prev_time = curr_time

        # extraction of pitch angle
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf.transformations.euler_from_quaternion(q)
        angle_pitch = euler[1]

        if (self.init):
            # at 1st execution, init
            self.startup_pitch = angle_pitch
            self.init = False

        pitch = (angle_pitch - self.startup_pitch) * 180/math.pi
        r = angular_velocity.z * 180/math.pi

        e_pitch =  self.desired_val - pitch
        if e_pitch <= -180:
            e_pitch = e_pitch + 360
        if e_pitch > 180:
            e_pitch = e_pitch - 360
        
        if dt == curr_time:
            return
        
        # Control:
        self.controller.set_step(dt)
        control_effort = -self.controller.control(e_pitch, r)
        # print(f"desired_pitch = {self.desired_val}")
        # print(f"pitch = {pitch}")
        # print(f"err = {e_pitch}")
        # print(f"pitch_control_effort = {control_effort}")
        msg = Float64MultiArray()
        msg.data = [control_effort, pitch]
        self.pub.publish(msg)

if __name__ == "__main__":
  rospy.init_node('pitch_controller_node')
  controller = YawController()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
