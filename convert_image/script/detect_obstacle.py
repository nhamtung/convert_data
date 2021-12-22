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

OFFSET = 12
TYPE_DEPTH_IMAGE = "16UC1"
TYPE_COLOR_IMAGE = "bgr8"

path_package = ""
x_mouse = 0
y_mouse = 0
cameraInfo = None
depth_image_rotate_resize = None
node_name = ""

use_detect = True
use_rotate_90_counter_clockwise = True
is_display_origin_color_image = False
is_display_origin_depth_image = False
is_display_resize_depth_image = True
camera = "/camera"
topic_camera_info_sub = "/color/camera_info"
topic_color_image_sub = "/color/image_raw"
topic_depth_image_sub = "/depth/image_rect_raw"
distance_field_detect = 2
distance_field_warning = 1.4
distance_field_dangerous = 0.7
top_image_resize = 240
bottom_image_resize = 240
left_image_resize = 424
right_image_resize = 424

color_image_origin_name = camera + "/color_image_origin"
depth_image_origin_name = camera + "/depth_image_origin"
depth_image_resize_name = camera + "/depth_image_resize"
topic_color_image_pub = "/color_image_message"
topic_depth_image_pub = "/depth_image_message"

scale_depth_color_width = 0.0
scale_depth_color_height = 0.0

class get_distance_object_from_camera:
  def __init__(self):
    global node_name, path_package, camera, topic_camera_info_sub, topic_color_image_sub, topic_depth_image_sub, topic_color_image_pub, topic_depth_image_pub
    global is_display_origin_color_image, is_display_origin_depth_image, is_display_resize_depth_image
    self.bridge = CvBridge()
    node_name = rospy.get_name()
    self.getParam(node_name)

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
    global cameraInfo, use_rotate_90_counter_clockwise, use_detect
    global depth_image_rotate_resize, scale_depth_color_width, scale_depth_color_height
    global path_package, color_image_origin_name, depth_image_origin_name, depth_image_resize_name
    global x_mouse, y_mouse, top_image_resize, bottom_image_resize, left_image_resize, right_image_resize
    try:
      if cameraInfo is None:
        cameraInfo = camera_info

      depth_image = self.bridge.imgmsg_to_cv2(depth_data, TYPE_DEPTH_IMAGE)
      cv_rgb = self.bridge.imgmsg_to_cv2(rgb_data, TYPE_COLOR_IMAGE)

      scale_depth_color_width = float(cv_rgb.shape[1])/float(depth_image.shape[1])
      scale_depth_color_height = float(cv_rgb.shape[0])/float(depth_image.shape[0])
      # print("cv_rgb.shape: ", cv_rgb.shape)
      # print("depth_image.shape: ", depth_image.shape)

      top_image = 0
      bottom_image = 0
      left_image = 0
      right_image = 0
      origin_width = 0
      origin_height = 0
      if use_rotate_90_counter_clockwise:
        cv_rgb_rotate = cv2.rotate(cv_rgb, cv2.ROTATE_90_COUNTERCLOCKWISE)
        depth_image_rotate = cv2.rotate(depth_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        top_image = right_image_resize
        bottom_image = left_image_resize
        left_image = top_image_resize
        right_image = bottom_image_resize
        template = scale_depth_color_width
        scale_depth_color_width = scale_depth_color_height
        scale_depth_color_height = template
      else:
        cv_rgb_rotate = cv_rgb
        depth_image_rotate = depth_image
        top_image = top_image_resize
        bottom_image = bottom_image_resize
        left_image = left_image_resize
        right_image = right_image_resize

      origin_width = depth_image_rotate.shape[1]/2
      origin_height = depth_image_rotate.shape[0]/2
      # rospy.loginfo("scale_depth_color_width: %f", scale_depth_color_width)
      # rospy.loginfo("scale_depth_color_height: %f", scale_depth_color_height)

      top_color_image = int(scale_depth_color_height*top_image)
      bottom_color_image = int(scale_depth_color_height*bottom_image)
      left_color_image = int(scale_depth_color_width*left_image)
      right_color_image = int(scale_depth_color_width*right_image)
      origin_color_width = int(scale_depth_color_width*origin_width)
      origin_color_height = int(scale_depth_color_height*origin_height)
      cv_rgb_rotate_resize = self.resize(cv_rgb_rotate, top_color_image, bottom_color_image, left_color_image, right_color_image, origin_color_width, origin_color_height)
      depth_image_rotate_resize = self.resize(depth_image_rotate, top_image, bottom_image, left_image, right_image, origin_width, origin_height)
      # rospy.loginfo("origin_color_width = %d, origin_color_height = %d", origin_color_width, origin_color_height)
      # rospy.loginfo("origin_width = %d, origin_height = %d", origin_width, origin_height)
      # rospy.loginfo("top_color_image = %d, bottom_color_image = %d, left_color_image = %d, right_color_image = %d", top_color_image, bottom_color_image, left_color_image, right_color_image)
      # rospy.loginfo("top_image = %d, bottom_image = %d, left_image = %d, right_image = %d", top_image, bottom_image, left_image, right_image)
      # rospy.loginfo("cv_rgb_rotate_resize: width = %d, height = %d", cv_rgb_rotate_resize.shape[1], cv_rgb_rotate_resize.shape[0])
      # rospy.loginfo("depth_image_rotate: width = %d, height = %d", depth_image_rotate.shape[1], depth_image_rotate.shape[0])
      # rospy.loginfo("depth_image_rotate_resize: width = %d, height = %d", depth_image_rotate_resize.shape[1], depth_image_rotate_resize.shape[0])
      cv2.setMouseCallback(depth_image_resize_name, self.mouseEvent)
  
      prior_time = time.time()
      detect_point, warning_point, dangerous_point = self.detectObstacle(use_detect, depth_image_rotate_resize, top_image, bottom_image, left_image, right_image)
      rospy.loginfo("Time to check: %f", time.time()-prior_time) 

      if is_display_origin_color_image:
        if len(detect_point) != 0:
          for i in range(0, len(detect_point)):
            if i%2 == 0:
              center_x, center_y = self.revertPoint(cv_rgb_rotate_resize, depth_image_rotate, depth_image_rotate_resize, detect_point[i], detect_point[i+1])
              cv2.circle(cv_rgb_rotate_resize, (center_x, center_y), 3, (0, 255, 0), -1)
        if len(warning_point) != 0:
          for i in range(0, len(warning_point)):
            if i%2 == 0:
              center_x, center_y = self.revertPoint(cv_rgb_rotate_resize, depth_image_rotate, depth_image_rotate_resize, warning_point[i], warning_point[i+1])
              cv2.circle(cv_rgb_rotate_resize, (center_x, center_y), 3, (0, 255, 255), -1)
        if len(dangerous_point) != 0:
          for i in range(0, len(dangerous_point)):
            if i%2 == 0:
              center_x, center_y = self.revertPoint(cv_rgb_rotate_resize, depth_image_rotate, depth_image_rotate_resize, dangerous_point[i], dangerous_point[i+1])
              cv2.circle(cv_rgb_rotate_resize, (center_x, center_y), 3, (0, 0, 255), -1)

        cv2.circle(cv_rgb_rotate_resize, (cv_rgb_rotate_resize.shape[1]/2, cv_rgb_rotate_resize.shape[0]/2), 3, (0, 0, 0), -1)
        center_start_x, center_start_y = self.revertPoint(cv_rgb_rotate_resize, depth_image_rotate, depth_image_rotate_resize, 0, 0)
        center_end_x, center_end_y = self.revertPoint(cv_rgb_rotate_resize, depth_image_rotate, depth_image_rotate_resize, depth_image_rotate_resize.shape[1], depth_image_rotate_resize.shape[0])
        cv2.rectangle(cv_rgb_rotate_resize, (center_start_x, center_start_y), (center_end_x, center_end_y), (255, 255, 0), 3)

      # self.mouseDistance(cv_rgb_rotate_resize, depth_image_rotate, depth_image_rotate_resize, OFFSET, x_mouse, y_mouse)
    except CvBridgeError as e:
      print(e)

    try:
      if is_display_origin_color_image:
        self.showImage(color_image_origin_name, cv_rgb_rotate)
      if is_display_origin_depth_image:
        self.showImage(depth_image_origin_name, depth_image_rotate)
      if is_display_resize_depth_image:
        self.showImage(depth_image_resize_name, depth_image_rotate_resize)

      depth_image_message = self.bridge.cv2_to_imgmsg(depth_image_rotate_resize, TYPE_DEPTH_IMAGE)
      color_image_message = self.bridge.cv2_to_imgmsg(cv_rgb_rotate, TYPE_COLOR_IMAGE)
      self.depth_image_message_pub.publish(depth_image_message)
      self.color_image_message_pub.publish(color_image_message)
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

  def getParam(self, node_name):
    global use_detect, use_rotate_90_counter_clockwise, is_display_origin_color_image, is_display_origin_depth_image, is_display_resize_depth_image
    global camera, topic_camera_info_sub, topic_color_image_sub, topic_depth_image_sub
    global distance_field_detect, distance_field_warning, distance_field_dangerous
    global top_image_resize, bottom_image_resize, left_image_resize, right_image_resize
    use_detect = rospy.get_param(node_name + "/use_detect")
    use_rotate_90_counter_clockwise = rospy.get_param(node_name + "/use_rotate_90_counter_clockwise")
    is_display_origin_color_image = rospy.get_param(node_name + "/is_display_origin_color_image")
    is_display_origin_depth_image = rospy.get_param(node_name + "/is_display_origin_depth_image")
    is_display_resize_depth_image = rospy.get_param(node_name + "/is_display_resize_depth_image")
    camera = rospy.get_param(node_name + "/camera")
    topic_camera_info_sub = rospy.get_param(node_name + "/topic_camera_info_sub")
    topic_color_image_sub = rospy.get_param(node_name + "/topic_color_image_sub")
    topic_depth_image_sub = rospy.get_param(node_name + "/topic_depth_image_sub")
    top_image_resize = rospy.get_param(node_name + "/top_image_resize")
    bottom_image_resize = rospy.get_param(node_name + "/bottom_image_resize")
    left_image_resize = rospy.get_param(node_name + "/left_image_resize")
    right_image_resize = rospy.get_param(node_name + "/right_image_resize")
    distance_field_detect = rospy.get_param(node_name + "/distance_field_detect")
    distance_field_warning = rospy.get_param(node_name + "/distance_field_warning")
    distance_field_dangerous = rospy.get_param(node_name + "/distance_field_dangerous")
    # rospy.loginfo("top_image_resize = %d, bottom_image_resize = %d, left_image_resize = %d, right_image_resize = %d", top_image_resize, bottom_image_resize, left_image_resize, right_image_resize)

  def showImage(self, window_name, cv_image):
    try:
      # rospy.loginfo("%s.width: %d , %s.height: %d", window_name, cv_image.shape[1], window_name, cv_image.shape[0])
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

  def resize(self, image, top_image, bottom_image, left_image, right_image, origin_x, origin_y):
    image_resize = image
    if (top_image+bottom_image)%2 != 0 or (left_image+right_image)%2 != 0:
      if top_image%2 != 0:
        top_image = top_image-1
      if bottom_image%2 != 0:
        bottom_image = bottom_image-1
      if left_image%2 != 0:
        left_image = left_image-1
      if right_image%2 != 0:
        right_image = right_image-1
    if self.checkSize(image, top_image, bottom_image, left_image, right_image, origin_x, origin_y):
      # rospy.loginfo("top_image = %d, bottom_image = %d, left_image = %d, right_image = %d", top_image, bottom_image, left_image, right_image)
      image_resize = image[origin_y-top_image:origin_y+bottom_image, origin_x-left_image:origin_x+right_image]
    return image_resize
  
  def checkSize(self, image, top_image, bottom_image, left_image, right_image, origin_x, origin_y):
    is_good = True
    if (top_image+bottom_image)%2 != 0 or (left_image+right_image)%2 != 0:
      is_good = False
      rospy.logerr("Can not resize image: (top_image_resize + bottom_image_resize) or (left_image_resize + right_image_resize) must be even number")
    elif left_image > origin_x or right_image > origin_x:
      is_good = False
      rospy.logerr("Can not resize image: left_image_resize (%d) or right_image_resize (%d) must be less than %d", left_image, right_image, origin_x)
    elif top_image > origin_y or bottom_image > origin_y:
      is_good = False
      rospy.logerr("Can not resize image: top_image_resize (%d) or bottom_image_resize (%d) must be less than %d", top_image, bottom_image, origin_y)
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
        
  def mouseDistance(self, cv_rgb, depth_image, depth_image_resize, offset, x_mouse, y_mouse):
    global scale_depth_color_width, scale_depth_color_height
    roi_depth = depth_image_resize[y_mouse:y_mouse+offset, x_mouse:x_mouse+offset]
    point_x, point_y, point_z, dist = self.getDistance(roi_depth)
    if not dist is None:
      point_x_str = "x: " + str(format(point_x, '.2f')) + "m"
      point_y_str = "y: " + str(format(point_y, '.2f')) + "m"
      point_z_str = "z: " + str(format(point_z, '.2f')) + "m"
      dist_str    = "d: " + str(format(dist   , '.2f')) + "m"

      center_x, center_y = self.revertPoint(cv_rgb, depth_image, depth_image_resize, x_mouse, y_mouse)

      if is_display_origin_color_image:
        cv2.circle(cv_rgb, (center_x, center_y), 3, (255, 255, 255), -1)
        cv2.putText(cv_rgb, point_x_str, (center_x+10, center_y-80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3, cv2.LINE_AA)
        cv2.putText(cv_rgb, point_y_str, (center_x+10, center_y-50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3, cv2.LINE_AA)
        cv2.putText(cv_rgb, point_z_str, (center_x+10, center_y-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3, cv2.LINE_AA)
        cv2.putText(cv_rgb, dist_str, (center_x+10, center_y+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3, cv2.LINE_AA)

  def revertPoint(self, cv_rgb, depth_image, depth_image_resize, x_point, y_point):
    center_x = 0
    center_y = 0
    center_x_ = int((depth_image.shape[1]/2-depth_image_resize.shape[1]/2+x_point))
    if x_point < depth_image_resize.shape[1]/2:
      center_x = cv_rgb.shape[1]/2-int(abs(depth_image.shape[1]/2-center_x_)*scale_depth_color_width)
    else:
      center_x = cv_rgb.shape[1]/2+int(abs(depth_image.shape[1]/2-center_x_)*scale_depth_color_width)
    center_y_ = int((depth_image.shape[0]/2-depth_image_resize.shape[0]/2+y_point))
    if y_point < depth_image_resize.shape[0]/2:
      center_y = cv_rgb.shape[0]/2-int(abs(depth_image.shape[0]/2-center_y_)*scale_depth_color_height)
    else:
      center_y = cv_rgb.shape[0]/2+int(abs(depth_image.shape[0]/2-center_y_)*scale_depth_color_height)
    return center_x, center_y

  def detectObstacleThread(self):
    global OFFSET, use_detect
    global depth_image_rotate_resize
    global top_image, bottom_image, left_image, right_image
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
      # prior_time = time.time()
      # self.detectObstacle(use_detect, depth_image_rotate_resize, top_image, bottom_image, left_image, right_image)
      # rospy.loginfo("Time to check: %f", time.time()-prior_time) 
      rate.sleep() 
  def detectObstacle(self, use_detect, depth_image, top_image, bottom_image, left_image, right_image):
    global OFFSET, node_name, distance_field_detect, distance_field_warning, distance_field_dangerous
    detect_point = []
    warning_point = []
    dangerous_point = []
    if use_detect:
      for i in range(0, left_image+right_image):
        for j in range(0, top_image+bottom_image):
          if i%OFFSET is 0 and j%OFFSET is 0:
            if not depth_image is None:
              roi_depth = depth_image[j:j+OFFSET, i:i+OFFSET]
              point_x_, point_y_, point_z_, dist = self.getDistance(roi_depth)
              if point_y_ is not None:
                # rospy.loginfo("point_y_: %f", point_y_)
                if dist>distance_field_warning and dist<distance_field_detect and point_y_<0:
                  detect_point.append(i)
                  detect_point.append(j)
                elif dist>distance_field_dangerous and dist<distance_field_warning and point_y_<0:
                  warning_point.append(i)
                  warning_point.append(j)
                elif dist>0.1 and dist<distance_field_dangerous and point_y_<0:
                  dangerous_point.append(i)
                  dangerous_point.append(j)
            else:
              rospy.loginfo("depth_image_rotate_resize is None")
      num_dangerous = 0
      num_warning = 0
      num_detect  = 0
      if len(dangerous_point)>100:
        # rospy.loginfo(node_name + " - len(dangerous_point): %d", len(dangerous_point))
        for i in range(0,len(dangerous_point)-6, 6):
          # rospy.loginfo(node_name + "dangerous_point[i+2]-dangerous_point[i  ]-OFFSET: %d", dangerous_point[i+2]-dangerous_point[i]-OFFSET)
          # rospy.loginfo(node_name + "dangerous_point[i+4]-dangerous_point[i+2]-OFFSET: %d", dangerous_point[i+4]-dangerous_point[i+2]-OFFSET)
          # rospy.loginfo(node_name + "dangerous_point[i+6]-dangerous_point[i+4]-OFFSET: %d", dangerous_point[i+6]-dangerous_point[i+4]-OFFSET)
          if dangerous_point[i+2]-dangerous_point[i]-OFFSET < 2 and dangerous_point[i+2]-dangerous_point[i]-OFFSET >= 0:
            if dangerous_point[i+4]-dangerous_point[i+2]-OFFSET < 2 and dangerous_point[i+4]-dangerous_point[i+2]-OFFSET >= 0:
              if dangerous_point[i+6]-dangerous_point[i+4]-OFFSET < 2 and dangerous_point[i+6]-dangerous_point[i+4]-OFFSET >= 0:
                num_dangerous = num_dangerous + 1
        if num_dangerous>0:
          rospy.logerr(node_name + " - Dangerous Obstacle: %d", num_dangerous)
        for i in range(0, len(detect_point)):
          detect_point.pop(0)
        for i in range(0, len(warning_point)): 
          warning_point.pop(0)
      elif len(warning_point)>50:
        rospy.loginfo(node_name + " - len(warning_point): %d", len(warning_point))
        for i in range(0,len(warning_point)-6, 6):
          if warning_point[i+2]-warning_point[i]-OFFSET < 2 and warning_point[i+2]-warning_point[i]-OFFSET >= 0:
            if warning_point[i+4]-warning_point[i+2]-OFFSET < 2 and warning_point[i+4]-warning_point[i+2]-OFFSET >= 0:
              if warning_point[i+6]-warning_point[i+4]-OFFSET < 2 and warning_point[i+6]-warning_point[i+4]-OFFSET >= 0:
                num_warning = num_warning + 1
        if num_warning>0:
          rospy.logwarn(node_name + " - Warning Obstacle: %d", num_warning)
        for i in range(0, len(detect_point)):
          detect_point.pop(0)
        for i in range(0, len(dangerous_point)):
          dangerous_point.pop(0)
      elif len(detect_point)>30:
        rospy.loginfo(node_name + " - len(detect_point): %d", len(detect_point))
        for i in range(0,len(detect_point)-6, 6):
          if detect_point[i+2]-detect_point[i]-OFFSET < 2 and detect_point[i+2]-detect_point[i]-OFFSET >= 0:
            if detect_point[i+4]-detect_point[i+2]-OFFSET < 2 and detect_point[i+4]-detect_point[i+2]-OFFSET >= 0:
              if detect_point[i+6]-detect_point[i+4]-OFFSET < 2 and detect_point[i+6]-detect_point[i+4]-OFFSET >= 0:
                num_detect = num_detect + 1
        if num_detect>0:
          rospy.loginfo(node_name + " - Detect Obstacle: %d", num_detect)
        for i in range(0, len(warning_point)):
          warning_point.pop(0)
        for i in range(0, len(dangerous_point)):
          dangerous_point.pop(0)
      else:
        rospy.loginfo(node_name + " - ELSE")
        for i in range(0, len(detect_point)):
          detect_point.pop(0)
        for i in range(0, len(warning_point)):
          warning_point.pop(0)
        for i in range(0, len(dangerous_point)):
          dangerous_point.pop(0)
    return detect_point, warning_point, dangerous_point

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