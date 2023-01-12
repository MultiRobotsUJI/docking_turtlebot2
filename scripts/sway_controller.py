#!/usr/bin/env python
import rospy
from pid import PID
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from waterlinked_a50_ros_driver.msg import DVL
from alpha_beta_gamma_filter import alpha_beta_gamma_filter
from std_msgs.msg import Float64MultiArray
import tf
import math

from PI_Controller import*

class SwayController:

    def __init__(self):

        self.pub = rospy.Publisher(
            'controller/sway/effort', Float64MultiArray, queue_size=10)
        
        self.init = False

        self.startup_sway = 0.0
        self.desired_val = 0

        self.step = 0.02
        self.prev_time = 0
        self.alpha = 0.45
        self.beta = 0.1

        self.controller = PID()

        self.v_e0 = 0
        self.vd_e0 = 0

        self.get_params()
        self.set_controller()
        
        self.sensor_sub = rospy.Subscriber(
            "/dvl/data", DVL, self.sensor_callback)
        self.reset_sub = rospy.Subscriber(
            "controllers/reset", Empty, self.reset_callback)
        self.desired_val_sub = rospy.Subscriber(
            "controller/sway/desired", Float64, self.desired_val_callback)

    def set_controller(self):
        self.controller.set_gains(self.kp, self.ki, self.kd)
        self.controller.set_step(self.step)
        
    def desired_val_callback(self, msg):
        self.desired_val = msg.data

    def get_params(self):
        self.kp = rospy.get_param('controller/sway/kp', 0.0)
        self.ki = rospy.get_param('controller/sway/ki', 0.0)
        self.kd = rospy.get_param('controller/sway/kd', 0.0)

    def reset_callback(self, data):
        self.v_e0 = 0
        self.vd_e0 = 0
        self.controller.reset_controller()
        self.init = True

    def sensor_callback(self,data): 
        v = data.velocity.y  # Linear velocity along y (sway) 
        
        # update dt
        curr_time = rospy.Time.now().to_sec()
        dt = curr_time - self.prev_time
        self.prev_time = curr_time

        if dt == curr_time:
            return

        # Filter : 
        self.vd_e0, self.v_e0 = alpha_beta_gamma_filter(
            self.v_e0, self.vd_e0, 0, v, self.alpha, self.beta, dt)

        # Control:
        self.controller.set_step(dt)
        e_vel = self.desired_val - self.v_e0
        control_effort = self.controller.control(e_vel, self.vd_e0)
        msg = Float64MultiArray()
        msg.data = [control_effort, self.v_e0]
        self.pub.publish(Float64(control_effort))

if __name__ == "__main__":
  rospy.init_node('sway_controller_node')
  controller = SwayController()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
