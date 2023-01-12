#!/usr/bin/env python

# Reference: https://hal.inria.fr/hal-01355384/document

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import scipy
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
from camera_parameters import CameraParameters

import utils as ut
import math
from transform import Transform

class ArucoServoing:
    def __init__(self):
        self.cam = CameraParameters()
        self.transform = Transform()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5x5_100)
        self.aruco_params = aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.kp = 0.0001
        self.error_x_prev = 0
        self.error_y_prev = 0
        self.compressed_flag = True

        #Camera parameters
        # 813.124, 830.767, 585.5, 265
        self.camera_matrix = np.array([[813.124, 0, 320],[0,830.767, 224],[0,0,1]])
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.lambda_vs = 0.05 #Kp for visual servoing

        ## Aruco parameters:
        #26
        dock_goal = np.array([[[122., 207.],
        [169., 207.],
        [169., 253.],
        [122., 253.]]])
        #UNKNOWN
        butler_goal = np.array([[[122., 207.],
        [169., 207.],
        [169., 253.],
        [122., 253.]]])
        #UNKNOWN
        serve_goal = np.array([[[122., 207.],
        [169., 207.],
        [169., 253.],
        [122., 253.]]])

        rospy.Subscriber("/blue_rov1/CompressedImage", CompressedImage, self.image_callback)
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # self.goal_points = np.array([[509.5, 257.5], [107.5, 257.0]])
        # self.goal_points = np.array([[526., 192.], [567., 192.], [567., 232.], [526., 232.]])
        self.goal_points = np.array([[[89., 239.], [126., 239.], [126., 275.], [90., 275.]]])
        # self.goal_points = np.array([[75., 240.] , [129., 240.], [129., 294.],  [75.,294.]])
        # self.goal_points = np.array([[509.5, 257.5]])

    def detect_marker(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=parameters)
        corners_np = np.array(corners)
        frame_markers = aruco.drawDetectedMarkers(image, corners, ids)
        if len(corners) != 0:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, self.camera_matrix, self.dist_coeffs)
            _ = cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
            rvec = np.array(rvec).flatten()
            self.quat = self.transform.rodrigues_to_quaternion(rvec)
            self.transformed_matrix = self.transform.transform_position(tvec, self.quat, as_quat=True)

        cv2.imshow("image", image)
        cv2.waitKey(3)
        cv2.imshow("frame_markers", frame_markers)
        cv2.waitKey(3)

        aruco_centers = []
        for corner in corners:
            mid = [0, 0]
            #Computing the middle point of a square using two opposite corner's points
            for point in corner:
                mid[0] = (point[0][0] + point[2][0]) / 2
                mid[1] = (point[0][1] + point[2][1]) / 2
            # print("mid = {}".format(mid))
            aruco_centers.append(mid)
            cv2.circle(frame_markers, (int(mid[0]), int(mid[1])), 5, (0, 0, 255), -1) #-1 means filled circle
        cv2.imshow("frame_markers", frame_markers)
        cv2.waitKey(3)
        return frame_markers, aruco_centers, corners

    def image_callback(self, image_msg):
        if self.compressed_flag:
            try:
                np_arr = np.fromstring(image_msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
        else:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, "passthrough")
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
        self.image_height = cv_image.shape[0]
        self.image_width = cv_image.shape[1]
        frame_markers, aruco_centers, aruco_corners = self.detect_marker(cv_image)
        #Check if aruco is detected:
        if len(aruco_centers) == 0:
            self.cmd_vel_publisher.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
            return
        else:
            goal_points = self.goal_points.flatten()
            current_points = np.array(aruco_corners).flatten()
            print("current_points = {}".format(current_points))
            print("current_points_shape = {}".format(current_points.shape))
            print("goal_points = {}".format(goal_points))
            print("goal_points_shape = {}".format(goal_points.shape))
            # exit()
            velocity = self.velocity_controller(current_points, goal_points)
            vel_x, vel_y, vel_z, vel_roll, vel_pitch, vel_yaw = velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]
            vel_x, vel_y, vel_z = 0, 0, 0
            self.cmd_vel_publisher.publish(Twist(Vector3(float(vel_x), float(-vel_z), float(-vel_y)), Vector3(0, 0, 0)))


    def velocity_controller(self, current_points, desired_points_vs):
        current_points_meter = self.cam.convertListPoint2meter(current_points)
        desired_points_meter = self.cam.convertListPoint2meter(desired_points_vs)

        #compute vs error
        error_vs = np.zeros((1, 8))
        error_vs = current_points_meter - desired_points_meter
        print("error_vs = {}".format(error_vs))

        #compute interaction matrix in the FILE ./visual_servoig.py
        L = ut.interactionMatrixFeaturePoint2DList(current_points_meter)
        # TODO once it works with this matrix, change it for
        # 1. a rho tetha representation
        # 2. a segment representation

        #init the camera velocity
        vcam = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #TODO compute the velocity control law
        vcam_vs = -self.lambda_vs * np.linalg.pinv(L).dot(error_vs)
        vrobot = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        ## TODO find the control velocity expressed in the robot frame
        cam_to_rob_t = np.array([0, 0, 0])
        #Unity rotation matrix from camera to robot
        cam_to_rob_r = np.array([[0, 0, -1],
                                    [0, -1, 0],
                                    [-1, 0, 0]])
        twist_matrix = self.transform.velocityTwistMatrix(cam_to_rob_t[0], cam_to_rob_t[1], cam_to_rob_t[2],
                                            aRb=cam_to_rob_r)
        #print("pretty_boy_twist  = ", twist_matrix)
        vrobot = twist_matrix.dot(vcam_vs)
        return vrobot



if __name__ == "__main__":
    rospy.init_node("aruco_servoing")
    aruco_servoing = ArucoServoing()
    rospy.spin()

