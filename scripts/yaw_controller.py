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

        self.pub = rospy.Publisher('controller/yaw/effort', Float64MultiArray, queue_size=10)
        
        self.init = False

        self.startup_yaw = 0.0
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
            "controller/yaw/desired", Float64, self.desired_val_callback)

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
        self.kp = rospy.get_param('controller/yaw/kp', 0.0)
        self.ki = rospy.get_param('controller/yaw/ki', 0.0)
        self.kd = rospy.get_param('controller/yaw/kd', 0.0)
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

        # extraction of yaw angle
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf.transformations.euler_from_quaternion(q)
        angle_yaw = euler[2]

        if (self.init):
            # at 1st execution, init
            self.startup_yaw = angle_yaw
            self.init = False

        yaw = (angle_yaw - self.startup_yaw) * 180/math.pi
        r = angular_velocity.z * 180/math.pi

        e_yaw =  self.desired_val - yaw
        if e_yaw <= -180:
            e_yaw = e_yaw + 360
        if e_yaw > 180:
            e_yaw = e_yaw - 360
        
        if dt == curr_time:
            return
        
        # Control:
        self.controller.set_step(dt)
        control_effort = -self.controller.control(e_yaw, r)
        # print(f"desired_yaw = {self.desired_val}")
        # print(f"yaw = {yaw}")
        # print(f"err = {e_yaw}")
        # print(f"yaw_control_effort = {control_effort}")
        msg = Float64MultiArray()
        msg.data = [control_effort, yaw]
        self.pub.publish(msg)

if __name__ == "__main__":
  rospy.init_node('yaw_controller_node')
  controller = YawController()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
