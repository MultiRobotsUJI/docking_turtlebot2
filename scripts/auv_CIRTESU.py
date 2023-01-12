#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, Image, CompressedImage
from mavros_msgs.msg import StatusText, VFR_HUD
import cv2
from cv_bridge import CvBridge, CvBridgeError

class MainClass:
  def __init__(self) -> None:
    # Create publisher
    # self.publisher = rospy.Publisher("/test",String,queue_size=1)
    # Important variables
    self.imu_data = Imu()
    self.leak_data = StatusText()
    self.bridge = CvBridge()
    # Create subscribers
    rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
    rospy.Subscriber("/mavros/statustext/recv", StatusText, self.leak_callback)
    rospy.Subscriber("/camera0/image_raw", Image, self.image_callback)
    rospy.Subscriber("/camera0/image_raw/compressed", Image, self.compressed_image_callback)
    rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self.depth_callback)


  # Define IMU callback
  def imu_callback(self, imu_msg):
    self.imu_data = imu_msg
    # print(imu_msg)
  
  def leak_callback(self, leak_msg):
    self.leak_data = leak_msg
    # print(leak_msg)

  def image_callback(self, image_msg):
    # print(image_msg)
    # Try to convert the ROS Image message to a CV2 Image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image_msg, "passthrough")
      cv2.imshow("Image", cv_image)
      cv2.waitKey(3)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
  
  def compressed_image_callback(self, compressed_image_msg):
    # print(compressed_image_msg)
    pass
  
  def depth_callback(self, depth_msg):
    depth = depth_msg.altitude
    print("depth = {}".format(depth))
  
if __name__ == "__main__":
  # Initialize the node with rospy
  rospy.init_node('main_node')
  # Create an instance of the class
  main = MainClass()
  # spin to keep the script for exiting
  rospy.spin()
