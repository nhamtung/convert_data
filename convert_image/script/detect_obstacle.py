#!/usr/bin/env python
from __future__ import print_function

import roslib
import time
import sys
import threading
import multiprocessing
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

OFFSET = 4
TYPE_DEPTH_IMAGE = "16UC1"
TYPE_COLOR_IMAGE = "bgr8"

path_package = ""
x_mouse = 0
y_mouse = 0
cameraInfo = None
depth_image_rotate_resize = None

use_detect = True
use_rotate = True
is_display_origin_color_image = False
is_display_origin_depth_image = False
is_display_resize_depth_image = True
camera = "/camera"
topic_camera_info_sub = "/color/camera_info"
topic_color_image_sub = "/color/image_raw"
topic_depth_image_sub = "/depth/image_rect_raw"
distance_field_detect = 1.7
distance_field_warning = 1.2
distance_field_dangerous = 0.7
width_image_resize = 640
height_image_resize = 480

color_image_name = camera + "/color_image"
depth_image_name = camera + "/depth_image"
topic_color_image_pub = "/color_image_message"
topic_depth_image_pub = "/depth_image_message"

class get_distance_object_from_camera:
  def __init__(self):
    global path_package, camera, topic_camera_info_sub, topic_color_image_sub, topic_depth_image_sub, topic_color_image_pub, topic_depth_image_pub
    self.bridge = CvBridge()
    self.getParam()

    self.depth_image_message_pub = rospy.Publisher(camera + topic_color_image_pub , Image, queue_size=1)	
    rospy.loginfo("Publish the topic: " + camera + topic_color_image_pub)
    self.color_image_message_pub = rospy.Publisher(camera + topic_depth_image_pub , Image, queue_size=1)	
    rospy.loginfo("Publish the topic: " + camera + topic_depth_image_pub)

    self.camera_info_sub = message_filters.Subscriber(camera + topic_camera_info_sub , CameraInfo)
    rospy.loginfo("Subscriber the topic: " + camera + topic_camera_info_sub)
    self.image_sub = message_filters.Subscriber(camera + topic_color_image_sub , Image)
    rospy.loginfo("Subscriber the topic: " + camera + topic_color_image_sub)
    self.depth_sub = message_filters.Subscriber(camera + topic_depth_image_sub , Image)
    rospy.loginfo("Subscriber the topic: " + camera + topic_depth_image_sub)
        
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.camera_info_sub], queue_size=10, slop=0.5)
    self.ts.registerCallback(self.cameraCallback)

    rospack = rospkg.RosPack()
    path_package = rospack.get_path('convert_image')
    rospy.loginfo("path to package convert_image: " + path_package)
    
    # t = multiprocessing.Process(target=self.detectObstacleThread)
    # t = threading.Thread(target=self.detectObstacleThread)
    # t.start()
    # t.join()  # wait until threads finish their job

  def cameraCallback(self, rgb_data, depth_data, camera_info):
    global TYPE_COLOR_IMAGE, TYPE_DEPTH_IMAGE, OFFSET
    global cameraInfo, use_rotate, use_detect
    global depth_image_rotate_resize
    global path_package, color_image_name, depth_image_name
    global x_mouse, y_mouse, x_obstacle, y_obstacle, width_image_resize, height_image_resize
    try:
      if cameraInfo is None:
        cameraInfo = camera_info

      depth_image = self.bridge.imgmsg_to_cv2(depth_data, TYPE_DEPTH_IMAGE)
      cv_rgb = self.bridge.imgmsg_to_cv2(rgb_data, TYPE_COLOR_IMAGE)

      width_image = 0
      height_image = 0
      origin_width = 0
      origin_height = 0
      if use_rotate:
        cv_rgb = cv2.rotate(cv_rgb, cv2.ROTATE_90_COUNTERCLOCKWISE)
        depth_image_rotate = cv2.rotate(depth_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        width_image = height_image_resize
        height_image = width_image_resize
        origin_width = depth_image.shape[0]/2
        origin_height = depth_image.shape[1]/2
      else:
        depth_image_rotate = depth_image
        width_image = width_image_resize
        height_image = height_image_resize
        origin_width = depth_image.shape[1]/2
        origin_height = depth_image.shape[0]/2
      # rospy.loginfo("width_image = %d", width_image)
      # rospy.loginfo("height_image = %d", height_image)

      depth_image_rotate_resize = self.resize(depth_image_rotate, width_image, height_image, origin_width, origin_height)

      cv2.setMouseCallback(depth_image_name, self.mouseEvent)
      cv2.circle(depth_image_rotate_resize, (x_mouse, y_mouse), 3, (0, 0, 255), -1)
      cv2.circle(cv_rgb, (depth_image.shape[1]/2-width_image/2+x_mouse, depth_image.shape[0]/2-height_image/2+y_mouse), 3, (0, 0, 255), -1)
  
      prior_time = time.time()
      x_obstacle, y_obstacle = self.detectObstacle(use_detect, depth_image_rotate_resize, width_image, height_image)
      cv2.circle(cv_rgb, (depth_image.shape[1]/2-width_image/2+x_obstacle, depth_image.shape[0]/2-height_image/2+y_obstacle), 3, (255, 0, 255), -1)
      rospy.loginfo("Time to check: %f", time.time()-prior_time) 

      self.mouseDistance(cv_rgb, depth_image, OFFSET, x_mouse, y_mouse)
    except CvBridgeError as e:
      print(e)

      
    try:
      depth_image_message = self.bridge.cv2_to_imgmsg(depth_image_rotate_resize, TYPE_DEPTH_IMAGE)
      color_image_message = self.bridge.cv2_to_imgmsg(cv_rgb, TYPE_COLOR_IMAGE)
      self.depth_image_message_pub.publish(depth_image_message)
      self.color_image_message_pub.publish(color_image_message)
      self.showRosImage(color_image_name, cv_rgb, TYPE_COLOR_IMAGE)
      self.showRosImage(depth_image_name, depth_image_rotate_resize, TYPE_DEPTH_IMAGE)
    except CvBridgeError as e:
      print(e)
      
  def getCameraInfo(self):
    # cameraInfo_K = np.array(cameraInfo.K)
    # Intrinsic camera matrix for the raw (distorted) images.
    #     [fx  0 cx]
    # K = [ 0 fy cy]
    #     [ 0  0  1]
    global cameraInfo
    try:
      if not cameraInfo is None:
        m_fx = cameraInfo.K[0]
        m_fy = cameraInfo.K[4]
        m_cx = cameraInfo.K[2]
        m_cy = cameraInfo.K[5]
        inv_fx = 1. / m_fx
        inv_fy = 1. / m_fy
        return m_cx, m_cy, inv_fx, inv_fy
    except:
      print("Something went wrong in camera_info")

  def getParam(self):
    global path_package, camera, topic_camera_info_sub, topic_color_image_sub, topic_depth_image_sub, topic_color_image_pub, topic_depth_image_pub
    camera = "/camera"
    topic_camera_info_sub = "/color/camera_info"
    topic_color_image_sub = "/color/image_raw"
    topic_depth_image_sub = "/depth/image_rect_raw"

  def showImage(self, window_name, cv_image, type_image):
    try:
      # cv_image = self.bridge.imgmsg_to_cv2(image_ros_data, type_image)
      cv2.imshow(window_name, cv_image)
      cv2.waitKey(30)
    except:
      print("Something went wrong when show image")

  def mouseEvent(self, event, x, y, flags, param):
    global x_mouse, y_mouse, use_detect
    try:
      if event == cv2.EVENT_MOUSEMOVE:
        x_mouse, y_mouse = x, y
      # elif event == cv2.EVENT_LBUTTONDOWN:
      #   use_detect = False
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

  def getDistance(self, roi_depth):
    n = 0
    sum = 0
    dist = None
    point_x = None 
    point_y = None 
    point_z = None 
    m_cx, m_cy, inv_fx, inv_fy = self.getCameraInfo()
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
    return point_x, point_y, point_z, dist
        
  def mouseDistance(self, cv_rgb, depth_image, offset, x_mouse, y_mouse):
    global depth_image_rotate_resize
    roi_depth = depth_image_rotate_resize[y_mouse:y_mouse+offset, x_mouse:x_mouse+offset]
    point_x, point_y, point_z, dist = self.getDistance(roi_depth)
    if not dist is None:
      point_x_str = str(format(point_x, '.2f')) + "m"
      point_y_str = str(format(point_y, '.2f')) + "m"
      point_z_str = str(format(point_z, '.2f')) + "m"
      dist_str = str(format(dist, '.2f')) + "m"
      cv2.putText(depth_image_rotate_resize, dist_str, (x_mouse+5, y_mouse), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3, cv2.LINE_AA)
      cv2.putText(cv_rgb, point_x_str, (depth_image.shape[1]/2-width_image_resize/2+x_mouse+5, depth_image.shape[0]/2-height_image_resize/2+y_mouse), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3, cv2.LINE_AA)
      cv2.putText(cv_rgb, point_y_str, (depth_image.shape[1]/2-width_image_resize/2+x_mouse+5, depth_image.shape[0]/2-height_image_resize/2+y_mouse+30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3, cv2.LINE_AA)
      cv2.putText(cv_rgb, point_z_str, (depth_image.shape[1]/2-width_image_resize/2+x_mouse+5, depth_image.shape[0]/2-height_image_resize/2+y_mouse+60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3, cv2.LINE_AA)
      cv2.putText(cv_rgb, dist_str, (depth_image.shape[1]/2-width_image_resize/2+x_mouse+5, depth_image.shape[0]/2-height_image_resize/2+y_mouse+90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3, cv2.LINE_AA)

  def detectObstacleThread(self):
    global OFFSET, use_detect, x_obstacle, y_obstacle
    global depth_image_rotate_resize
    global width_image_resize, height_image_resize
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
      # prior_time = time.time()
      # self.detectObstacle(use_detect, depth_image_rotate_resize, width_image_resize, height_image_resize)
      # rospy.loginfo("Time to check: %f", time.time()-prior_time) 
      rate.sleep() 
  def detectObstacle(self, use_detect, depth_image, width_image, height_image):
    global field_detect, field_warning, field_dangerous
    x_obstacle = 0 
    y_obstacle = 0
    if use_detect:
      num_detect = 0
      num_warning = 0
      num_dangerous = 0
      for i in range(0, width_image):
        for j in range(0, height_image):
          if j>height_image/2+25:
            break
          if i%OFFSET is 0 and j%OFFSET is 0:
            if not depth_image is None:
              roi_depth = depth_image[j:j+OFFSET, i:i+OFFSET]
              point_x, point_y, point_z, dist = self.getDistance(roi_depth)
              if dist>field_warning and dist<field_detect:
                num_detect = num_detect+1
              elif dist>field_dangerous and dist<field_warning:
                num_warning = num_warning+1
              elif dist>0.1 and dist<field_dangerous:
                num_dangerous = num_dangerous+1
            else:
              rospy.loginfo("depth_image_rotate_resize is None")
      # print(num_dangerous, num_warning, num_detect)
      if num_dangerous>50:
        rospy.logerr("Dangerous Obstacle: %d", num_dangerous)
      elif num_warning>30:
        rospy.logwarn("Warning Obstacle: %d", num_warning)
      elif num_detect>10:
        rospy.loginfo("Detect Obstacle: %d", num_detect)
    return x_obstacle, y_obstacle

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