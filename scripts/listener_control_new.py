#!/usr/bin/env python
from doctest import master
import math
from os import kill
import string

import numpy as np
from yaml import FlowEntryToken
import rospy
import tf
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from waterlinked_a50_ros_driver.msg import DVL
from waterlinked_a50_ros_driver.msg import DVLBeam
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist
from autonomous_rov.msg import pwm
from alpha_beta_gamma_filter import alpha_beta_gamma_filter
from PI_Controller import*
# from brping import Ping1D
import time
import sys
import argparse
###---- Visual Tracking and Servoing----
from transform import velocityTwistMatrix
from transform import homogenousMatrix
import visual_servoing as vs
import camera_parameters as cam


# ---------- Global Variables ---------------

set_mode = [0]*3
# Vmax_mot = 1900
# Vmin_mot = 1100

Vmax_mot = 1600
Vmin_mot = 1400

# Conditions
arming = False
set_mode[0] = True   # Mode manual
set_mode[1] = False  # Mode automatic without correction
set_mode[2] = False  # Mode with correction

# visual servoing
global desired_points_vs
global vcam_vs
global lambda_vs
global n_points_vs

vcam_vs = np.array([0, 0, 0, 0, 0, 0])
lambda_vs = np.array([0.5,
                      0.5,
                      0.5,
                      0.25,
                      0.5,
                      0.1])  # yaw
n_points_vs = 8
desired_points_vs = []
enable_vs = 0

roll_left_right = 1500
yaw_left_right = 1500
ascend_descend = 1500
forward_reverse = 1500
lateral_left_right = 1500
pitch_left_right = 1500


def trackercallback(data):
    global desired_points_vs
    global n_points_vs
    global roll_left_right
    global yaw_left_right
    global ascend_descend
    global forward_reverse
    global lateral_left_right
    global pitch_left_right

    # read the current tracked point
    current_points = data.data

    # if we have current_points of same size as desired_points
    # then we can compute control law
    print("n_d_points = ", len(desired_points_vs))
    print("n_c_points = ", len(current_points))
    if(len(current_points) > 0 and
       len(desired_points_vs) == len(current_points)):

        #convert points from pixels to meters
        current_points_meter = cam.convertListPoint2meter(current_points)
        desired_points_meter = cam.convertListPoint2meter(desired_points_vs)

        #compute vs error
        error_vs = np.zeros((1, 16))
        error_vs = current_points_meter - desired_points_meter

        #compute interaction matrix in the FILE ./visual_servoig.py
        L = vs.interactionMatrixFeaturePoint2DList(current_points_meter)
        # TODO once it works with this matrix, change it for
        # 1. a rho tetha representation
        # 2. a segment representation

        #init the camera velocity
        vcam = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #TODO compute the velocity control law
        vcam_vs = -lambda_vs * np.linalg.pinv(L).dot(error_vs)

        ## Find the relative robot/camera position
        ## You can train in the file testTransform.py
        ## robot frame        |  camera frame
        ##                    |
        ##    ------> x       |  -----> z
        ##    |               |  |
        ##    |               |  |
        ##    v z             |  v y
        ##                    |

        print('Visual servoing : vcam =', vcam_vs)
        vrobot = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        ## TODO find the control velocity expressed in the robot frame
        cam_to_rob_t = np.array([0.15, 0, 0])
        cam_to_rob_r = np.array([[0, 0, 1],
                                 [1, 0, 0],
                                 [0, 1, 0]])
        twist_matrix = velocityTwistMatrix(cam_to_rob_t[0], cam_to_rob_t[1], cam_to_rob_t[2],
                                           aRb=cam_to_rob_r)
        #print("pretty_boy_twist  = ", twist_matrix)
        vrobot = twist_matrix.dot(vcam_vs)

        print('Then vrobot =  ', vrobot)

        vel = Twist()
        vel.angular.x = vrobot[3]
        vel.angular.y = vrobot[4]
        vel.angular.z = vrobot[5]
        vel.linear.x = vrobot[0]
        vel.linear.y = vrobot[1]
        vel.linear.z = vrobot[2]
        # publish the visual servoing velocity
        pub_visual_servoing_vel.publish(vel)

        # publish the error
        error_vs_msg = Float64MultiArray(data=error_vs)
        pub_visual_servoing_err.publish(error_vs_msg)

        print("press A to launch the visual servoing control")
        if (set_mode[2]):

            # Extract cmd_vel message
            # FIXME be carreful of the sign may depends on your robot
            # roll_left_right = mapValueScalSat(vel.angular.x)
            # yaw_left_right = map_vel_yaw(vel.angular.z)
            # yaw_left_right = mapValueScalSat(vel.angular.z)#-1
            ascend_descend = mapValueScalSat(-1*vel.linear.z) # controlled by pressure
            forward_reverse = mapValueScalSat(vel.linear.x)
            lateral_left_right = mapValueScalSat(vel.linear.y)#-1
            # pitch_left_right = mapValueScalSat(vel.angular.y)#1
            roll_left_right = 1500
            yaw_left_right = 1500
            # ascend_descend = 1500
            # forward_reverse = 1500
            pitch_left_right = 1500
            # lateral_left_right = 1500
            print("pwms = ", pitch_left_right, roll_left_right, ascend_descend,
                                    yaw_left_right, forward_reverse, lateral_left_right)
            
            # setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend,
            #                 yaw_left_right, forward_reverse, lateral_left_right)


def desiredpointscallback(data):
   # print( "desired point callback")
    global desired_points_vs
    desired_points_vs = data.data




def joyCallback(data):
    global arming
    global set_mode

    # Joystick buttons
    btn_arm = data.buttons[7]           # Start button
    btn_disarm = data.buttons[6]        # Back button
    btn_manual_mode = data.buttons[3]    # Y button
    btn_automatic_mode = data.buttons[2]  # X button
    btn_corrected_mode = data.buttons[0]  # A button

    # Disarming when Back button is pressed
    if (btn_disarm == 1 and arming == True):
        arming = False
        armDisarm(arming)

    # Arming when Start button is pressed
    if (btn_arm == 1 and arming == False):
        arming = True
        armDisarm(arming)

    # Switch manual and auto mode
    if (btn_manual_mode and not set_mode[0]):
        set_mode[0] = True
        set_mode[1] = False
        set_mode[2] = False
        rospy.loginfo("Mode manual")
    if (btn_automatic_mode and not set_mode[1]):
        set_mode[0] = False
        set_mode[1] = True
        set_mode[2] = False
        rospy.loginfo("Mode automatic")
    if (btn_corrected_mode and not set_mode[2]):
        # set sum errors to 0 here, ex: Sum_Errors_Vel = [0]*3
        set_mode[0] = False
        set_mode[1] = False
        set_mode[2] = True
        rospy.loginfo("Mode correction")


def armDisarm(armed):
    # This functions sends a long command service with 400 code to arm or disarm motors
    if (armed):
        rospy.wait_for_service('mavros/cmd/command')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            armService(0, 400, 0, 1, 0, 0, 0, 0, 0, 0)
            rospy.loginfo("Arming Succeeded")
        except (rospy.ServiceException, e):
            rospy.loginfo("Except arming")
    else:
        rospy.wait_for_service('mavros/cmd/command')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            armService(0, 400, 0, 0, 0, 0, 0, 0, 0, 0)
            rospy.loginfo("Disarming Succeeded")
        except (rospy.ServiceException, e):
            rospy.loginfo("Except disarming")


def velCallback(cmd_vel):

    global set_mode
    # Only continue if manual_mode is enabled
    if (set_mode[1] or set_mode[2]):
        return

    # Extract cmd_vel message
    roll_left_right = mapValueScalSat(-cmd_vel.angular.x)
    yaw_left_right = mapValueScalSat(cmd_vel.angular.z)
    ascend_descend = mapValueScalSat(-cmd_vel.linear.z)
    forward_reverse = mapValueScalSat(-cmd_vel.linear.x)
    lateral_left_right = mapValueScalSat(cmd_vel.linear.y)
    pitch_left_right = mapValueScalSat(-cmd_vel.angular.y)

    setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend,
                    yaw_left_right, forward_reverse, lateral_left_right)

def map_vel_yaw(value, b=None):
    global Vmax_mot
    global Vmin_mot
    if (value >= 0):
        m = 400   # Slope of the positive force linear function
        b = 1536 if b is None else b
    else:
        m = 400  # Slope of the negtaive force linear function
        b = 1464 if b is None else b
        
    pulse_width = value * m + b

    # On limite la commande en vitesse
    if pulse_width > Vmax_mot:
        pulse_width = Vmax_mot
    if pulse_width < Vmin_mot:
        pulse_width = Vmin_mot
    
    return pulse_width
    

def mapValueScalSat(value):
    global Vmax_mot
    global Vmin_mot
    # Correction_Vel and joy between -1 et 1
    # scaling for publishing with setOverrideRCIN values between 1100 and 1900
    # neutral point is 1500
    pulse_width = value * 400 + 1500

    # On limite la commande en vitesse
    if pulse_width > Vmax_mot:
        pulse_width = Vmax_mot
    if pulse_width < Vmin_mot:
        pulse_width = Vmin_mot

    return pulse_width


def setOverrideRCIN(channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
    # This function replaces setservo for motor commands.
    # It overrides Rc channels inputs and simulates motor controls.
    # In this case, each channel manages a group of motors not individually as servo set

    msg_override = OverrideRCIn()

    msg_override.channels[0] = np.uint(
        channel_pitch)       # pulseCmd[4]--> pitch
    msg_override.channels[1] = np.uint(
        channel_roll)        # pulseCmd[3]--> roll
    msg_override.channels[2] = np.uint(
        channel_throttle)    # pulseCmd[2]--> heave
    msg_override.channels[3] = np.uint(channel_yaw)        # pulseCmd[5]--> yaw

    msg_override.channels[4] = np.uint(
        channel_forward)     # pulseCmd[0]--> surge

    msg_override.channels[5] = np.uint(
        channel_lateral)     # pulseCmd[1]--> sway
    msg_override.channels[6] = 1500
    msg_override.channels[7] = 1500

    # print("<3=====D ",msg_override)
    pub_msg_override.publish(msg_override)


def DoThing(msg):
    print(msg.data)
    setOverrideRCIN(1500, 1500, 1500, 1500, msg.data, 1500)


class Master:
    def __init__(self):
        
        self.surge_vel_setpoint_pub = rospy.Publisher(
            "controller/surge_vel/desired", Float64, queue_size=10)
        self.depth_setpoint_pub = rospy.Publisher("controller/depth/desired",Float64,queue_size=10)
        self.yaw_setpoint_pub = rospy.Publisher("controller/yaw/desired",Float64,queue_size=10)
        self.surge_setpoint_pub = rospy.Publisher("controller/surge/desired",Float64,queue_size=10)
        self.sway_setpoint_pub = rospy.Publisher("controller/sway/desired", Float64, queue_size=10)
        self.pitch_setpoint_pub = rospy.Publisher(
            "controller/pitch/desired", Float64, queue_size=10)


        # Surge Mode (vel from dvl vs Pos from pinger)
        self.use_surge_vel = False
        self.drowned = False
        
        # PWMs
        self.surge_vel_pwm = 1500
        self.surge_pwm = 1500
        self.depth_pwm = 1500
        self.sway_pwm = 1500
        self.yaw_pwm = 1500
        
        # Mission Thresholds
        self.free_dist_thresh = 1000
        self.max_depth_err = 0.1
        self.max_wall_dist_err = 100
        
        # Setpoints
        self.depth_desired = 0.3 #0.3
        self.yaw_desired = 0.0
        self.pitch_desired = 0.0
        self.surge_desired = 700
        self.surge_vel_desired = 0.0
        self.surge_vel_nominal = 0.1
        self.sway_desired = 0.0
        
        # Initial Values of Control Variables
        self.surge = 0
        self.surge_vel = 0
        self.yaw = 0
        self.pitch = 0
        self.sway = 0
        self.depth = 0
        
        # self.state_names = ["drown", "go_to_wall", "keep_to_wall", "search"]
        # self.state_nums = {name:val for val, name in self.state_names.items()}
        self.actions = {"drown": self.drown_action,
                        "go_to_wall": self.go_to_wall_action,
                        "keep_to_wall": self.keep_to_wall_action,
                        "search": self.search_action,
                        "yaw":self.yaw_action,
                        "bonus": self.bonus_action,
                        "drown_vs": self.drown_vs_action,
                        "vs": self.vs_action}
        
        self.state = "drown_vs"
        
        rospy.sleep(2)
        
        self.send_setpoints(depth=self.depth_desired, yaw=self.yaw_desired,
                            surge=self.surge_desired, sway=self.sway_desired, pitch=self.pitch_desired)
        
        rospy.Subscriber("controller/surge_vel/effort",
                         Float64MultiArray, self.surge_vel_cb)
        rospy.Subscriber("controller/surge/effort",
                         Float64MultiArray, self.surge_cb)
        rospy.Subscriber("controller/yaw/effort",
                         Float64MultiArray, self.yaw_cb)
        rospy.Subscriber("controller/pitch/effort",
                         Float64MultiArray, self.pitch_cb)
        rospy.Subscriber("controller/depth/effort",
                         Float64MultiArray, self.depth_cb)
        rospy.Subscriber("controller/sway/effort",
                         Float64MultiArray, self.sway_cb)

    def surge_cb(self, msg):
        self.surge_pwm = self.PWM_Cmd(msg.data[0], b=1500)
        self.surge = msg.data[1]
        self.update_state()
    
    def surge_vel_cb(self, msg):
        self.surge_vel_pwm = self.PWM_Cmd(msg.data[0])
        self.surge_vel = msg.data[1]
        self.update_state()

    def yaw_cb(self, msg):
        self.yaw_pwm = self.PWM_Cmd(msg.data[0])
        self.yaw = msg.data[1]
        self.update_state()
    
    def pitch_cb(self,msg):
        # global pitch_left_right
        self.pitch_pwm = self.PWM_Cmd(msg.data[0], b=1500)
        self.pitch = msg.data[1]
        # pitch_left_right = self.pitch_pwm
        self.update_state()

    def depth_cb(self, msg):
        global ascend_descend
        self.depth_pwm = self.PWM_Cmd(msg.data[0])
        ascend_descend = self.depth_pwm
        self.depth = msg.data[1]
        self.update_state()

    def sway_cb(self, msg):
        self.sway_pwm = self.PWM_Cmd(msg.data[0])
        self.sway = msg.data[1]
        self.update_state()

    # Function used to calculate the necessary PWM for each motor

    def PWM_Cmd(self, thrust_req, b = None):
        if (thrust_req >= 0):
            m = 86.93393326839376   # Slope of the positive force linear function
            b = 1536 if b is None else b
        else:
            m = 110.918185437553874  # Slope of the negtaive force linear function
            b = 1464 if b is None else b

        PWM = int(m * thrust_req/4) + b
        if PWM > Vmax_mot:
            PWM = Vmax_mot
        if PWM < Vmin_mot:
            PWM = Vmin_mot
        return PWM

    def send_setpoints(self, depth, yaw, surge, sway,pitch=0, surge_vel=0):
        self.surge_vel_setpoint_pub.publish(Float64(surge_vel))
        self.depth_setpoint_pub.publish(Float64(depth))
        self.yaw_setpoint_pub.publish(Float64(yaw))
        self.surge_setpoint_pub.publish(Float64(surge))
        self.sway_setpoint_pub.publish(Float64(sway))
        self.pitch_setpoint_pub.publish(Float64(pitch))
    
    def update_state(self):
        if set_mode[2]:
            self.actions[self.state]()
            # print(f"state = {self.state}")
        if abs(abs(self.depth) - self.depth_desired) > self.max_depth_err and not self.drowned:
            # self.state = "drown"
            self.state = "drown_vs"
            return
        elif self.state == "drown_vs":
            self.state = "vs"
            # self.drowned = True
            return

        elif self.state == "drown":
            # if abs(abs(self.depth) - self.depth_desired) <= self.max_depth_err:
            self.state = "go_to_wall"
            return
        
        elif self.state == "go_to_wall":
            print(f"cond = {abs(self.surge - self.surge_desired)}")
            if abs(self.surge - self.surge_desired) < self.max_wall_dist_err:
                self.state = "keep_to_wall"
            return
        
        elif self.state == "keep_to_wall":
            # self.state = "search"
            self.state = "bonus"
            return
        
        elif self.state == "search":
            if self.surge >= self.free_dist_thresh:
                self.state = "go_to_wall"
            return
        elif self.state == "bonus":
            self.state = "go_to_wall"
            return
        elif self.state == "yaw":
            return    

    def drown_action(self):
        # Correct sway, yaw, and depth, but not surge distance, keep rov in a vertical column
        setOverrideRCIN(1500, 1500, self.depth_pwm,
                        self.yaw_pwm, 1500, self.sway_pwm)
    
    def drown_vs_action(self):
        # Correct sway, yaw, and depth, but not surge distance, keep rov in a vertical column
        # print(self.depth_pwm)
        setOverrideRCIN(1500, 1500, self.depth_pwm,
                        1500, 1500, 1500)
    
    def vs_action(self):
        global roll_left_right
        global yaw_left_right
        global ascend_descend
        global forward_reverse
        global lateral_left_right
        global pitch_left_right
        # Correct sway, yaw, and depth, but not surge distance, keep rov in a vertical column
        # setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend,
        #                 yaw_left_right, forward_reverse, lateral_left_right)
        setOverrideRCIN(self.pitch_pwm, roll_left_right, ascend_descend,
                        self.yaw_pwm, forward_reverse, lateral_left_right)
    
    def yaw_action(self):
        # Correct sway, yaw, and depth, but not surge distance, keep rov in a vertical column
        setOverrideRCIN(1500, 1500, self.depth_pwm,
                        self.yaw_pwm, 1500, 1500)
    
    def go_to_wall_action(self):
        # Start moving forward, while keeping depth, sway, and yaw positions
        self.surge_vel_desired = self.surge_vel_nominal
        self.send_setpoints(depth=self.depth_desired, yaw=self.yaw_desired,
                            surge=self.surge_desired, sway=self.sway_desired,
                            surge_vel=self.surge_vel_desired)
        surge_pwm = self.surge_vel_pwm if self.use_surge_vel else self.surge_pwm
        print(f"surge_pwm = {surge_pwm}")
        setOverrideRCIN(1500, 1500, self.depth_pwm,
                        self.yaw_pwm, surge_pwm, self.sway_pwm)
        
    def keep_to_wall_action(self):
        # stop moving, and keep depth, yaw, surge, and sway constant.
        self.surge_vel_desired = 0.0
        self.send_setpoints(depth=self.depth_desired, yaw=self.yaw_desired,
                            surge=self.surge_desired, sway=self.sway_desired,
                            surge_vel=self.surge_vel_desired)
        surge_pwm = self.surge_vel_pwm if self.use_surge_vel else self.surge_pwm
        print(f"surge_pwm = {surge_pwm}")
        setOverrideRCIN(1500, 1500, self.depth_pwm,
                        self.yaw_pwm, surge_pwm, self.sway_pwm)
    
    def bonus_action(self):
        # stop moving, and keep depth, yaw, surge, and sway constant.
        surge_pwm = self.surge_vel_pwm if self.use_surge_vel else self.surge_pwm
        setOverrideRCIN(1500, 1500, self.depth_pwm,
                        self.yaw_pwm, surge_pwm, 1510)
    
    def search_action(self):
        # Rotate in yaw while keeping depth value, no sway or surge control.
        self.yaw_desired -= 0.05 # in degrees
        if self.yaw_desired > 180:
            self.yaw_desired = self.yaw_desired - 360
        if self.yaw_desired <= -180:
            self.yaw_desired = self.yaw_desired + 360
        self.send_setpoints(depth=self.depth_desired, yaw=self.yaw_desired,
                    surge=self.surge_desired, sway=self.sway_desired)
        setOverrideRCIN(1500, 1500, self.depth_pwm, self.yaw_pwm, 1500, 1500)


def subscriber():
    rospy.Subscriber("joy", Joy, joyCallback)
    rospy.Subscriber("cmd_vel", Twist, velCallback)
    rospy.Subscriber("do/thing", Int16, DoThing)
    #camera
    rospy.Subscriber("tracked_points",Float64MultiArray,trackercallback, queue_size=1)
    rospy.Subscriber("desired_points",Float64MultiArray,desiredpointscallback, queue_size=1)
    rospy.spin()  # Execute subscriber in loop


if __name__ == '__main__':

    armDisarm(False)  # Not automatically disarmed at startup
    rospy.init_node('autonomous_MIR', anonymous=False)
        
    pub_msg_override = rospy.Publisher(
        "mavros/rc/override", OverrideRCIn, queue_size=10, tcp_nodelay=True)
    pub_angle_degre = rospy.Publisher(
        'angle_degree', Twist, queue_size=10, tcp_nodelay=True)
    pub_depth = rospy.Publisher(
        'depth/state', Float64, queue_size=10, tcp_nodelay=True)
    pub_angular_velocity = rospy.Publisher(
        'angular_velocity', Twist, queue_size=10, tcp_nodelay=True)
    pub_linear_vel = rospy.Publisher(
        'linear_velocity', Twist, queue_size=10, tcp_nodelay=True)
    
    pub_visual_servoing_vel = rospy.Publisher('visual_servoing_velocity', Twist, queue_size = 10, tcp_nodelay = True)
    pub_visual_servoing_err = rospy.Publisher("visual_servoing_error",Float64MultiArray,queue_size=1,tcp_nodelay = True)

    
    master = Master()

    subscriber()
