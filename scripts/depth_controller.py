#!/usr/bin/env python
import rospy
from pid import PID
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import FluidPressure
import tf
import math

from PI_Controller import*
from alpha_beta_gamma_filter import alpha_beta_gamma_filter


class DepthController:

    def __init__(self):

        self.pub = rospy.Publisher('controller/depth/effort', Float64MultiArray, queue_size=10)
        
        self.init = False
        self.depth_wrt_startup = 0
        self.desired_val = 0

        self.rho = 1000.0
        self.gravity = 9.80665
        self.step = 0.02
        self.prev_time = 0
        self.depth_p0 = 0
        
        # Filter gains :
        self.alpha = 0.45
        self.beta = 0.1
        self.zdot_est, self.z_est = 0, 0  # Estimated surge velocity/position

        self.controller = PID()

        self.get_params()
        self.set_controller()
        
        self.sensor_sub = rospy.Subscriber(
            "mavros/imu/water_pressure", FluidPressure, self.sensor_callback)
        self.reset_sub = rospy.Subscriber(
            "controllers/reset", Empty, self.reset_callback)
        self.desired_val_sub = rospy.Subscriber(
            "controller/depth/desired", Float64, self.desired_val_callback)

    def set_controller(self):
        self.controller.set_gains(self.kp, self.ki, self.kd)
        self.controller.set_step(self.step)
        
    def desired_val_callback(self, msg):
        self.desired_val = msg.data
        print("recieved_desired")

    def get_params(self):
        self.g = rospy.get_param('controller/depth/g', 0.0)
        self.kp = rospy.get_param('controller/depth/kp', 0.0)
        self.ki = rospy.get_param('controller/depth/ki', 0.0)
        self.kd = rospy.get_param('controller/depth/kd', 0.0)
        print(f"kp = {self.kp}")

    def reset_callback(self, data):
        self.controller.reset_controller()
        self.init = True

    def sensor_callback(self, data):
        # get data
        pressure = data.fluid_pressure
        
        # update dt
        curr_time = rospy.Time.now().to_sec()
        dt = curr_time - self.prev_time
        self.prev_time = curr_time
        
        depth = (pressure - 101300)/(self.rho * self.gravity)
        
        if (self.init):
            # 1st execution, init
            self.depth_p0 = depth
            self.init = False
        
        if dt == curr_time:
            return
                
        self.depth_wrt_startup = depth - self.depth_p0
        
        # Filter:
        self.zdot_est, self.z_est = alpha_beta_gamma_filter(
            self.z_est, self.zdot_est, 0, self.depth_wrt_startup, self.alpha, self.beta, dt)
        
        # Control:
        self.controller.set_step(dt)
        e = self.desired_val - self.z_est
        control_effort = self.controller.control(e, self.zdot_est, bias=self.g)
        # print(f"depth = {self.depth_wrt_startup}")
        # print(f"z_est = {self.z_est}")
        # print(f"err = {e}")
        # print(f"depth_control_effort = {control_effort}")
        
        # pub
        msg = Float64MultiArray()
        msg.data = [control_effort, self.z_est]
        self.pub.publish(msg)

if __name__ == "__main__":
  rospy.init_node('depth_controller_node')
  controller = DepthController()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
