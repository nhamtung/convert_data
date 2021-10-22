#include "ros/ros.h"
#include <string> 
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <iostream>

using namespace std;
using namespace cv;

ros::Publisher depth_image_convert_pub, color_image_convert_pub;
std::string CAMERA_INFO_TOPIC, DEPTH_IMAGE_TOPIC, COLOR_IMAGE_TOPIC;
uint16_t x_mouse, y_mouse;
float m_fx, m_fy, m_cx, m_cy, inv_fx, inv_fy;

void mouseCallback(int  event, int  x, int  y, int  flag, void *param){
    if (event == EVENT_MOUSEMOVE) {
        x_mouse = x;
        y_mouse = y;
        // ROS_INFO("(x_mouse, y_mouse) = (%d, %d)", x_mouse, y_mouse);
    }
}

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html

    // ROS_INFO_THROTTLE(1, "camera_height: %d", msg->height);
    // ROS_INFO_THROTTLE(1, "camera_width: %d", msg->width);
    // ROS_INFO_THROTTLE(1, "camera_distortion_model: %s", msg->distortion_model.c_str());
    // ROS_INFO_THROTTLE(1, "camera_binning_x: %d", msg->binning_x);
    // ROS_INFO_THROTTLE(1, "camera_binning_y: %d", msg->binning_y);
    // ROS_INFO_THROTTLE(1, "camera_roi_x_offset: %d", msg->roi.x_offset);
    // ROS_INFO_THROTTLE(1, "camera_roi_y_offset: %d", msg->roi.y_offset);
    // ROS_INFO_THROTTLE(1, "camera_roi_height: %d", msg->roi.height);
    // ROS_INFO_THROTTLE(1, "camera_roi_width: %d", msg->roi.width);
    // ROS_INFO_THROTTLE(1, "camera_roi_do_rectify: %d", msg->roi.do_rectify);

    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    m_fx = msg->K[0];
    m_fy = msg->K[4];
    m_cx = msg->K[2];
    m_cy = msg->K[5];
    inv_fx = 1/m_fx;
    inv_fy = 1/m_fy;
}

void colorImageCallback(const sensor_msgs::Image::ConstPtr& msg){
    // ROS_INFO_THROTTLE(1, "color_image_height: %d", msg->height);
    // ROS_INFO_THROTTLE(1, "color_image_width: %d", msg->width);
    // ROS_INFO_THROTTLE(1, "color_image_encoding: %s", msg->encoding.c_str());
    // ROS_INFO_THROTTLE(1, "color_image_is_bigendian: %d", msg->is_bigendian);
    // ROS_INFO_THROTTLE(1, "color_image_step: %d", msg->step);
    // ROS_INFO_THROTTLE(1, "depth_image_data_size: %d", msg->data.size());
    
    // cv::circle(cv_msg->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    // cv::drawMarker(cv_msg->image, cv::Point(cv_msg->image.cols/2, cv_msg->image.rows/2), cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);

    sensor_msgs::Image color_image;
    color_image.header = msg->header;
    color_image.height = msg->height;
    color_image.width = msg->width;
    color_image.encoding = msg->encoding;
    color_image.is_bigendian = msg->is_bigendian;
    color_image.step = msg->step;
    for (uint32_t i=0; i<msg->data.size(); i++){
        if(i%1==0){
            color_image.data.push_back(msg->data[i]);
        }
    }
    color_image_convert_pub.publish(color_image);

    cv_bridge::CvImagePtr cv_msg = cv_bridge::toCvCopy(color_image, "bgr8");
    cv::imshow("convert_color_image", cv_msg->image);
    cv::waitKey(3);
}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg){
    // ROS_INFO_THROTTLE(1, "depth_image_height: %d", msg->height);
    // ROS_INFO_THROTTLE(1, "depth_image_width: %d", msg->width);
    // ROS_INFO_THROTTLE(1, "depth_image_encoding: %s", msg->encoding.c_str());
    // ROS_INFO_THROTTLE(1, "depth_image_is_bigendian: %d", msg->is_bigendian);
    // ROS_INFO_THROTTLE(1, "depth_image_step: %d", msg->step);
    // ROS_INFO_THROTTLE(1, "depth_image_data_size: %d", msg->data.size());

    sensor_msgs::Image depth_image;
    depth_image.header = msg->header;
    depth_image.height = msg->height;
    depth_image.width = msg->width;
    depth_image.encoding = msg->encoding;
    depth_image.is_bigendian = msg->is_bigendian;
    depth_image.step = msg->step;
    for (uint32_t i=0; i<msg->data.size(); i++){
        if(i%1==0){
            depth_image.data.push_back(msg->data[i]);
        }
    }
    depth_image_convert_pub.publish(depth_image);

    sensor_msgs::Image roi_depth;
    roi_depth.header = msg->header;
    roi_depth.height = msg->height;
    roi_depth.width = msg->width;
    roi_depth.encoding = msg->encoding;
    roi_depth.is_bigendian = msg->is_bigendian;
    roi_depth.step = msg->step;
    float distance;
    uint8_t offset = 1;
    uint32_t start_column, end_column, start_row, end_row;
    if(x_mouse>offset){
        start_column = x_mouse-offset;
    }else{
        start_column = 0;
    }
    if(x_mouse+offset < msg->width){
        end_column = x_mouse+offset;
    }else{
        end_column = msg->width;
    }
    if(y_mouse>offset){
        start_row = y_mouse-offset;
    }else{
        start_row = 0;
    }
    if(y_mouse+offset < msg->height){
        end_row = y_mouse+offset;
    }else{
        end_row = msg->height;
    }
    // ROS_INFO("(start_column, end_column) : (%d, %d)", start_column, end_column);
    // ROS_INFO("(start_row, end_row) : (%d, %d)", start_row, end_row);

    uint32_t row = 0;
    uint32_t column = 0;
    uint32_t sum = 0;
    uint8_t count = 0;
    float distance_average, dist;
    float point_x, point_y, point_z;
    std::string dist_str;
    for (uint32_t i=0; i<msg->data.size(); i++){
        column++;
        if(i%msg->step==0){
            // ROS_INFO("column = %d", column);
            column = 0;
            row++;
        }
        if(row>start_row && row<end_row && column>start_column*msg->step/msg->width && column<end_column*msg->step/msg->width){
            roi_depth.data.push_back(msg->data[i]);
            // ROS_INFO("focus data - i = %d", i);
            count++;
            if(i%2==0){
                distance = msg->data[i] | msg->data[i+1] << 8;
            }
            sum = sum + distance;
        }else{
            roi_depth.data.push_back(msg->data[0]);
        }
    }
    // ROS_INFO("row = %d", row);
    if(count>0){
        distance_average = sum/count; // distance in mm
        // ROS_INFO("distance_average = %.0f mm", distance_average);

        point_z = distance_average*0.001;  // distance in m
        point_x = (x_mouse - m_cx) * point_z * inv_fx;
        point_y = (y_mouse - m_cy) * point_z * inv_fy;

        dist = sqrt(point_x*point_x + point_y*point_y + point_z*point_z);
        ROS_INFO("dist = %f m", dist);
        dist_str = std::to_string(dist) + " m";
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_ptr exception: %s", e.what());
      return;
    }
    cv::Mat depth_colormap;
    cv::convertScaleAbs(cv_ptr->image, depth_colormap, 0.1);
    cv::applyColorMap(depth_colormap, depth_colormap, cv::COLORMAP_JET);
    cv::drawMarker(depth_colormap, cv::Point(x_mouse, y_mouse), cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
    cv::putText(depth_colormap, dist_str, cv::Point(x_mouse + 5, y_mouse), cv::FONT_ITALIC, 1.0, cv::Scalar(0,255,0), 3, cv::LINE_8, false);
    cv::imshow("convert_depth_image", depth_colormap);

    cv_bridge::CvImageConstPtr cv_focus;
    try{
      cv_focus = cv_bridge::toCvCopy(roi_depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_focus exception: %s", e.what());
      return;
    }
    cv::Mat focus_point;
    cv::convertScaleAbs(cv_focus->image, focus_point, 0.1);
    cv::applyColorMap(focus_point, focus_point, cv::COLORMAP_JET);
    // cv::imshow("focus_point", focus_point);
    cv::waitKey(10);
}

void loadParameters(ros::NodeHandle n, std::string node_name){
  n.param<std::string>(node_name + "/camera_info_topic", CAMERA_INFO_TOPIC, "/camera/color/camera_info");
  ROS_INFO("camera_info_topic: %s", CAMERA_INFO_TOPIC.c_str());
  n.param<std::string>(node_name + "/depth_image_topic", DEPTH_IMAGE_TOPIC, "/camera/depth/image_rect_raw");
  ROS_INFO("depth_image_topic: %s", DEPTH_IMAGE_TOPIC.c_str());
  n.param<std::string>(node_name + "/color_image_topic", COLOR_IMAGE_TOPIC, "/camera/color/image_raw");
  ROS_INFO("color_image_topic: %s", COLOR_IMAGE_TOPIC.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "convert_image");
    ros::NodeHandle n;

    cv::namedWindow("convert_color_image");
    cv::namedWindow("convert_depth_image");
    // cv::namedWindow("focus_point");

    std::string node_name = ros::this_node::getName();
    loadParameters(n, node_name);

    setMouseCallback("convert_depth_image", mouseCallback);

    depth_image_convert_pub = n.advertise<sensor_msgs::Image>("/depth_image_convert", 10);
    color_image_convert_pub = n.advertise<sensor_msgs::Image>("/color_image_convert", 10);

    ros::Subscriber camera_info_sub = n.subscribe(CAMERA_INFO_TOPIC, 10, cameraInfoCallback);
    ros::Subscriber depth_image_sub = n.subscribe(DEPTH_IMAGE_TOPIC, 10, depthImageCallback);
    ros::Subscriber color_image_sub = n.subscribe(COLOR_IMAGE_TOPIC, 10, colorImageCallback);
    
    ros::spin();
    cv::destroyWindow("convert_color_image");
    cv::destroyWindow("convert_depth_image");
    // cv::destroyWindow("focus_point");
    return 0;
}