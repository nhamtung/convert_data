#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import rospkg
import pyautogui
import argparse
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import math

TYPE_DEPTH_IMAGE = "16UC1"
TYPE_COLOR_IMAGE = "bgr8"

path_package = ""
color_image_name = "color_image"
depth_image_name = "depth_image"
x_mouse = 0
y_mouse = 0
width_image_resize = 320
height_image_resize = 240

class get_distance_object_from_camera:
  def __init__(self):
    global path_package
    self.bridge = CvBridge()
    
    self.pub = rospy.Publisher('/images', Image, queue_size=1)	

    self.camera_info_sub = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
    self.image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    self.depth_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
        
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.camera_info_sub], queue_size=10, slop=0.5)
    self.ts.registerCallback(self.cameraCallback)

    rospack = rospkg.RosPack()
    path_package = rospack.get_path('convert_image')
    rospy.loginfo("path to package convert_image: " + path_package)

  def cameraCallback(self, rgb_data, depth_data, camera_info):
    global TYPE_COLOR_IMAGE, TYPE_DEPTH_IMAGE
    global path_package, color_image_name, depth_image_name
    global x_mouse, y_mouse, width_image_resize, height_image_resize
    try:
      cv_rgb = self.bridge.imgmsg_to_cv2(rgb_data, TYPE_COLOR_IMAGE)
      depth_image = self.bridge.imgmsg_to_cv2(depth_data, TYPE_DEPTH_IMAGE)
      depth_image_resize = self.resize(depth_image, width_image_resize, height_image_resize, depth_image.shape[1]/2, depth_image.shape[0]/2)

      cv2.setMouseCallback(depth_image_name, self.mouseEvent)
      cv2.circle(depth_image_resize, (x_mouse, y_mouse), 3, (0, 0, 255), -1)
      cv2.circle(cv_rgb, (depth_image.shape[1]/2-width_image_resize/2+x_mouse, depth_image.shape[0]/2-height_image_resize/2+y_mouse), 3, (0, 0, 255), -1)
  
      offset = 4
      self.mouseDistance(camera_info, cv_rgb, depth_image, depth_image_resize, offset, x_mouse, y_mouse)
      self.detectObstacle(camera_info, cv_rgb, depth_image, depth_image_resize, offset)
    except CvBridgeError as e:
      print(e)
      
    try:
      depth_image_message = self.bridge.cv2_to_imgmsg(depth_image_resize, TYPE_DEPTH_IMAGE)
      color_image_message = self.bridge.cv2_to_imgmsg(cv_rgb, TYPE_COLOR_IMAGE)
      self.pub.publish(depth_image_message)
      self.showRosImage(color_image_name, color_image_message, TYPE_COLOR_IMAGE)
      self.showRosImage(depth_image_name, depth_image_message, TYPE_DEPTH_IMAGE)
    except CvBridgeError as e:
      print(e)
      
  def cameraInfo(self, camera_info):
    # camera_info_K = np.array(camera_info.K)
    # Intrinsic camera matrix for the raw (distorted) images.
    #     [fx  0 cx]
    # K = [ 0 fy cy]
    #     [ 0  0  1]
    try:
      m_fx = camera_info.K[0]
      m_fy = camera_info.K[4]
      m_cx = camera_info.K[2]
      m_cy = camera_info.K[5]
      inv_fx = 1. / m_fx
      inv_fy = 1. / m_fy
      return m_cx, m_cy, inv_fx, inv_fy
    except:
      print("Something went wrong in camera_info")

  def showRosImage(self, window_name, image_ros_data, type_image):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image_ros_data, type_image)
      cv2.imshow(window_name, cv_image)
      cv2.waitKey(30)
    except:
      print("Something went wrong when show image")

  def mouseEvent(self, event, x, y, flags, param):
    global x_mouse, y_mouse
    # rospy.loginfo("mouseEvent")
    try:
      if event == cv2.EVENT_MOUSEMOVE:
        x_mouse, y_mouse = x, y
        # print(x_mouse, y_mouse)
    except:
      print("Something went wrong when detect mouse event")

  def resize(self, depth_image, width_image_resize, height_image_resize, origin_x, origin_y):
    depth_image_resize = depth_image
    if self.checkSize(depth_image, width_image_resize, height_image_resize):
      depth_image_resize = depth_image[origin_y-height_image_resize/2:origin_y+height_image_resize/2, origin_x-width_image_resize/2:origin_x+width_image_resize/2]
    return depth_image_resize
  
  def checkSize(self, depth_image, width_image_resize, height_image_resize):
    is_good = True
    if width_image_resize%2 != 0 or height_image_resize%2 != 0:
      is_good = False
      rospy.loginfo("Can not resize image: width_image_resize or height_image_resize must be even number")
    elif width_image_resize > depth_image.shape[1]:
      is_good = False
      rospy.loginfo("Can not resize image: width_image_resize must be less than %d", depth_image.shape[1])
    elif height_image_resize > depth_image.shape[0]:
      is_good = False
      rospy.loginfo("Can not resize image: height_image_resize must be less than %d", depth_image.shape[0])
    return is_good

  def getDistance(self, camera_info, roi_depth):
    n = 0
    sum = 0
    dist = None
    m_cx, m_cy, inv_fx, inv_fy = self.cameraInfo(camera_info)
    for i in range(0,roi_depth.shape[0]):
        for j in range(0,roi_depth.shape[1]):
            value = roi_depth.item(i, j)
            # rospy.loginfo("value: %f", value)
            if value > 0.:
                n = n + 1
                sum = sum + value
    if n!=0:
      mean_z = sum/n
    
      point_z = mean_z * 0.001; # distance in meters
      point_x = (x_mouse - m_cx) * point_z * inv_fx
      point_y = (y_mouse - m_cy) * point_z * inv_fy
                
      dist = math.sqrt(point_x * point_x + point_y * point_y + point_z * point_z)
    return dist
        
  def mouseDistance(self, camera_info, cv_rgb, depth_image, depth_image_resize, offset, x_mouse, y_mouse):
    roi_depth = depth_image_resize[y_mouse:y_mouse+offset, x_mouse:x_mouse+offset]
    dist = self.getDistance(camera_info, roi_depth)
    if not dist is None:
      dist_str = str(format(dist, '.2f')) + "m"
      cv2.putText(depth_image_resize, dist_str, (x_mouse+5, y_mouse), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3, cv2.LINE_AA)
      cv2.putText(cv_rgb, dist_str, (depth_image.shape[1]/2-width_image_resize/2+x_mouse+5, depth_image.shape[0]/2-height_image_resize/2+y_mouse), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3, cv2.LINE_AA)

  def detectObstacle(self, camera_info, cv_rgb, depth_image, depth_image_resize, offset):
    roi_depth = depth_image_resize[y_mouse:y_mouse+offset, x_mouse:x_mouse+offset]
    dist = self.getDistance(camera_info, roi_depth)

def main(args):
  rospy.init_node('detect_object', anonymous=True)
  rospy.loginfo("get_distance_object_from_camera")
  dist_obj = get_distance_object_from_camera()
  cv2.destroyAllWindows()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)