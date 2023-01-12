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
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.aruco_params = aruco.DetectorParameters()
        self.bridge = CvBridge()
        self.kp = 0.001
        self.error_x_prev = 0
        self.error_y_prev = 0
        self.compressed_flag = True

        #Camera parameters
        # 813.124, 830.767, 585.5, 265
        self.camera_matrix = np.array([[813.124, 0, 320],[0,830.767, 224],[0,0,1]])
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.lambda_vs = 0.04 #Kp for visual servoing
        self.butler_goal = True
        self.end_goal_reached = False

        rospy.Subscriber("/agent_5/camera/rgb/image_raw/compressed", CompressedImage, self.image_callback)
        self.cmd_vel_publisher = rospy.Publisher("/agent_5/cmd_vel_mux/input/navi", Twist, queue_size=10)
        #74
        ## Aruco parameters:
        #UNKOWN
        dock_goal = np.array([[[122., 207.],
        [169., 207.],
        [169., 253.],
        [122., 253.]]])
        #26
        butler_goal = np.array([[[258.,  98.],
        [329.,  96.],
        [331., 165.],
        [260., 167.]]])
        #UNKNOWN
        serve_goal = np.array([[[122., 207.],
        [169., 207.],
        [169., 253.],
        [122., 253.]]])
        self.goals_dict = {'52' : dock_goal,
                           '26' : butler_goal,
                           '163' : serve_goal}

        self.current_goal = None

    def detect_marker(self, image):
        # detected_markers_dict = {k:None for k in self.goals_dict.keys()}
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        parameters = self.aruco_params
        all_arucos_corners, ids, _ = aruco.detectMarkers(gray_image, self.aruco_dict, parameters=parameters)
        print("ids = {}".format(ids))
        print("all_arucos_corners = {}".format(all_arucos_corners))
        frame_markers = aruco.drawDetectedMarkers(image, all_arucos_corners, ids)
        aruco_key_found = None
        detected_markers_dict = {}
        if len(all_arucos_corners) != 0:
            for id, one_aruco_corners in zip(ids, all_arucos_corners):
                if len(one_aruco_corners) == 1:
                    detected_markers_dict[str(id[0])] = one_aruco_corners
                    aruco_key_found = str(id[0])
                    # break
        if aruco_key_found is None:
            return None, None, None, None, None
        single_aruco_corners = detected_markers_dict[aruco_key_found]
        #Selecting the goal to track
        self.current_goal = self.goals_dict[aruco_key_found]
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(single_aruco_corners, 0.1, self.camera_matrix, self.dist_coeffs)
        _ = cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
        rvec = np.array(rvec).flatten()
        self.quat = self.transform.rodrigues_to_quaternion(rvec)
        self.transformed_matrix = self.transform.transform_position(tvec, self.quat, as_quat=True)

        cv2.imshow("frame_markers", frame_markers)
        cv2.waitKey(1)

        aruco_centers = []
        for corner in [single_aruco_corners]:
            mid = [0, 0]
            #Computing the middle point of a square using two opposite corner's points
            for point in corner:
                mid[0] = (point[0][0] + point[2][0]) / 2
                mid[1] = (point[0][1] + point[2][1]) / 2
            aruco_centers.append(mid)
            cv2.circle(frame_markers, (int(mid[0]), int(mid[1])), 5, (0, 0, 255), -1) #-1 means filled circle
        cv2.imshow("frame_markers", frame_markers)
        cv2.waitKey(3)
        return frame_markers, aruco_centers, single_aruco_corners, aruco_key_found, detected_markers_dict

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
        # cv2.imshow("image", cv_image)
        cv2.waitKey(3)
        frame_markers, aruco_centers, aruco_corners, aruco_key_found, detected_markers_dict = self.detect_marker(cv_image)
        print("aruco_key_found={}".format(aruco_key_found))
        #Check if aruco is detected:
        if aruco_corners is None:
            # keep_rotating : so the robot will keep searching for the aruco around itself
            keep_rotating = 0.5
            self.cmd_vel_publisher.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, keep_rotating)))
            return
        else:
            for k,v in detected_markers_dict.items():
                goal_points = self.goals_dict[k].flatten()
                current_points = np.array([v]).flatten()
            print("current_points = {}".format(current_points))
            print("goal_points = {}".format(goal_points))            
            velocity = self.velocity_controller(current_points, goal_points)
            # vel_x, vel_yaw = -velocity[0], velocity[5]
            vel_x, vel_y, vel_z, _, _, vel_yaw = -velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]
            err_1, err_2, err_3, err_4, err_5, err_6, err_7, err_8 \
             = self.error_vs[0], self.error_vs[1], self.error_vs[2],\
              self.error_vs[3], self.error_vs[4], self.error_vs[5], self.error_vs[6], self.error_vs[7]
            
            if abs(vel_y) < 0.01:
                # pass
                # vel_x = np.arctan2(vel_y, vel_x)*0.01
                vel_yaw = np.arctan2(vel_y, vel_x) * 0.01
                print("a7aaaaaaaaa")
            # else:
            #     vel_yaw = -vel_yaw
            #     vel_x = -0.1
            # elif np.any((err_1, err_3, err_5, err_7)) > 0.1 and np.any((err_2, err_4, err_6, err_8)) < 0.0:
            # if np.abs(err_1) - np.abs(err_2) > 0.5:
            #     vel_x = -0.05
            #     vel_yaw = -vel_yaw*5
            print("vel_x = {}".format(vel_x))
            print("vel_y = {}".format(vel_y))
            print("vel_z = {}".format(vel_z))
            print("vel_yaw = {}".format(vel_yaw))
            self.cmd_vel_publisher.publish(Twist(Vector3(float(vel_x), 0, 0), Vector3(0, 0, float(vel_yaw))))


    def velocity_controller(self, current_points, desired_points_vs):
        current_points_meter = self.cam.convertListPoint2meter(current_points)
        desired_points_meter = self.cam.convertListPoint2meter(desired_points_vs)
        print("current_points_meter = {}".format(current_points_meter))
        print("desired_points_meter = {}".format(desired_points_meter))

        #compute vs error
        self.error_vs = np.zeros((1, 8))
        self.error_vs = current_points_meter - desired_points_meter
        print("error_vs = {}".format(self.error_vs))

        #compute interaction matrix in the FILE ./visual_servoig.py
        L = ut.interactionMatrixFeaturePoint2DList(current_points_meter)

        #init the camera velocity
        vcam_vs = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #compute the velocity control law
        vcam_vs = -self.lambda_vs * np.linalg.pinv(L).dot(self.error_vs)
        vrobot = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        

        ## TODO find the control velocity expressed in the robot frame
        cam_to_rob_t = np.array([-0.15, 0, 0])
        # Rotation matrix from camera to robot
        # print("vcam_vs = {}".format(vcam_vs))
        cam_to_rob_r = np.array([[0, 0, -1],
                                    [1, 0, 0],
                                    [0, 1, 0]])
        twist_matrix = self.transform.velocityTwistMatrix(cam_to_rob_t[0], cam_to_rob_t[1], cam_to_rob_t[2],
                                            aRb=cam_to_rob_r)
        vrobot = twist_matrix.dot(vcam_vs)
        # print("vrobot = {}".format(vrobot))
        return vrobot



if __name__ == "__main__":
    rospy.init_node("aruco_servoing")
    aruco_servoing = ArucoServoing()
    rospy.spin()

