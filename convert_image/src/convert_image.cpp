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
double WIDTH_IMAGE_RESIZE, HEIGHT_IMAGE_RESIZE;
bool IS_DISPLAY_ORIGIN_COLOR_IMAGE, IS_DISPLAY_ORIGIN_DEPTH_IMAGE, IS_DISPLAY_RESIZE_DEPTH_IMAGE;
uint16_t x_mouse, y_mouse;
std::string name_color_window = "origin_color_image";
std::string name_depth_window = "origin_depth_image";
std::string name_resize_window = "resize_depth_image";
std::string name_dist_window = "dist_depth_image";
float m_fx, m_fy, m_cx, m_cy, inv_fx, inv_fy;
std::string dist_str;

bool checkSize(sensor_msgs::Image origin_depth_image, uint32_t width_image_resize, uint32_t height_image_resize);
sensor_msgs::Image resizeImage(sensor_msgs::Image origin_depth_image, uint32_t width_image_resize, uint32_t height_image_resize, double origin_x, double origin_y);
double getDistance(sensor_msgs::Image dist_image);
void showColorImage(std::string name_window, sensor_msgs::Image image, std::string type);
void showDepthImage(std::string name_window, sensor_msgs::Image image, std::string type);

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
    if(IS_DISPLAY_ORIGIN_COLOR_IMAGE){
        std::string type = sensor_msgs::image_encodings::BGR8;
        showColorImage(name_color_window, color_image, type);
    }
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
    depth_image.data = msg->data;
    if(IS_DISPLAY_ORIGIN_DEPTH_IMAGE){
        std::string type = sensor_msgs::image_encodings::TYPE_16UC1;
        cv::Mat depth_colormap;
        showDepthImage(name_depth_window, depth_image, type);
    }

    sensor_msgs::Image resize_image;
    if(checkSize(depth_image, WIDTH_IMAGE_RESIZE, HEIGHT_IMAGE_RESIZE)){
        resize_image = resizeImage(depth_image, WIDTH_IMAGE_RESIZE, HEIGHT_IMAGE_RESIZE, depth_image.width/2, depth_image.height/2);
        depth_image_convert_pub.publish(resize_image);
    }
    sensor_msgs::Image dist_image;
    uint8_t offset = 4;
    if(checkSize(resize_image, offset, offset)){
        dist_image = resizeImage(resize_image, offset, offset, x_mouse, y_mouse);
    }
    double dist = getDistance(dist_image);
    dist_str = std::to_string(dist) + " m";
    // if(IS_DISPLAY_RESIZE_DEPTH_IMAGE){
    //     std::string type = sensor_msgs::image_encodings::TYPE_16UC1;
    //     showDepthImage(name_dist_window, dist_image, type);
    // }
    if(IS_DISPLAY_RESIZE_DEPTH_IMAGE){
        std::string type = sensor_msgs::image_encodings::TYPE_16UC1;
        showDepthImage(name_resize_window, resize_image, type);
        setMouseCallback(name_resize_window, mouseCallback);
    }
}

bool checkSize(sensor_msgs::Image origin_depth_image, uint32_t width_image_resize, uint32_t height_image_resize){
    bool is_good = true;
    if(width_image_resize%2 != 0 || height_image_resize%2 != 0){
        is_good = false;
        ROS_ERROR("width_image_resize or height_image_resize must be even number");
    }else if(width_image_resize > origin_depth_image.width){
        is_good = false;
        ROS_ERROR("width_image_resize must be less than %d", origin_depth_image.width);
    }else if(height_image_resize > origin_depth_image.height){
        is_good = false;
        ROS_ERROR("height_image_resize must be less than %d", origin_depth_image.height);
    }
    return is_good;
}
sensor_msgs::Image resizeImage(sensor_msgs::Image origin_depth_image, uint32_t width_image_resize, uint32_t height_image_resize, double origin_x, double origin_y){
    sensor_msgs::Image image_resize;
    image_resize.header = origin_depth_image.header;
    image_resize.height = height_image_resize;
    image_resize.width = width_image_resize;
    image_resize.encoding = origin_depth_image.encoding;
    image_resize.is_bigendian = origin_depth_image.is_bigendian;
    image_resize.step = width_image_resize*origin_depth_image.step/origin_depth_image.width;

    uint32_t offset_width = width_image_resize/2;
    uint32_t offset_height = height_image_resize/2;
    uint32_t start_column, end_column, start_row, end_row;
    if(origin_x > offset_width){
        start_column = origin_x-offset_width;
    }else{
        start_column = 0;
    }
    if(origin_x+offset_width < origin_depth_image.width){
        end_column = origin_x+offset_width;
    }else{
        end_column = origin_depth_image.width;
    }
    if(origin_y > offset_height){
        start_row = origin_y-offset_height;
    }else{
        start_row = 0;
    }
    if(origin_y+offset_height < origin_depth_image.height){
        end_row = origin_y+offset_height;
    }else{
        end_row = origin_depth_image.height;
    }
    // ROS_INFO("(start_column, end_column) : (%d, %d)", start_column, end_column);
    // ROS_INFO("(start_row, end_row) : (%d, %d)", start_row, end_row);

    uint32_t row = 0;
    uint32_t column = 0;
    for (uint32_t i=0; i<origin_depth_image.data.size(); i++){
        column++;
        if(i%origin_depth_image.step==0){
            // ROS_INFO("column = %d", column);
            column = 0;
            row++;
        }
        if(row>=start_row && row<end_row && column>=start_column*origin_depth_image.step/origin_depth_image.width && column<end_column*origin_depth_image.step/origin_depth_image.width){
            image_resize.data.push_back(origin_depth_image.data[i]);
        }
    }
    return image_resize;
}
double getDistance(sensor_msgs::Image dist_image){
    double dist;
    uint32_t sum = 0;
    uint8_t count = 0;
    float distance, distance_average;
    float point_x, point_y, point_z;
    for(uint32_t i=0; i<dist_image.data.size(); i++){
        count++;
        if(i%2==0){
            distance = dist_image.data[i] | dist_image.data[i+1] << 8;
        }
        sum = sum + distance;
    }
    if(count>0){
        distance_average = sum/count; // distance in mm
        // ROS_INFO("distance_average = %.0f mm", distance_average);

        point_z = distance_average*0.001;  // distance in m
        point_x = (x_mouse - m_cx) * point_z * inv_fx;
        point_y = (y_mouse - m_cy) * point_z * inv_fy;

        dist = sqrt(point_x*point_x + point_y*point_y + point_z*point_z);
        ROS_INFO("dist = %f m", dist);
    }
    return dist;
}

void showColorImage(std::string name_window, sensor_msgs::Image image, std::string type){
    cv::namedWindow(name_window);
    cv_bridge::CvImagePtr cv_msg;
    try{
        cv_msg = cv_bridge::toCvCopy(image, type);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_msg exception: %s", e.what());
        return;
    }
    cv::imshow(name_window, cv_msg->image);
    cv::waitKey(3);
}
void showDepthImage(std::string name_window, sensor_msgs::Image image, std::string type){
    cv::namedWindow(name_window);
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(image, type);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_ptr exception: %s", e.what());
      return;
    }
    cv::Mat depth_colormap;
    cv::convertScaleAbs(cv_ptr->image, depth_colormap, 0.1);
    cv::applyColorMap(depth_colormap, depth_colormap, cv::COLORMAP_JET);
    cv::drawMarker(depth_colormap, cv::Point(x_mouse, y_mouse), cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
    cv::putText(depth_colormap, dist_str, cv::Point(x_mouse + 5, y_mouse), cv::FONT_ITALIC, 1.0, cv::Scalar(0,255,0), 3, cv::LINE_8, false);
    cv::imshow(name_window, depth_colormap);
    cv::waitKey(10);
}

void loadParameters(ros::NodeHandle n, std::string node_name){
  n.param<bool>(node_name + "/is_display_origin_color_image", IS_DISPLAY_ORIGIN_COLOR_IMAGE, true);
  ROS_INFO("is_display_origin_color_image: %d", IS_DISPLAY_ORIGIN_COLOR_IMAGE);
  n.param<bool>(node_name + "/is_display_origin_depth_image", IS_DISPLAY_ORIGIN_DEPTH_IMAGE, true);
  ROS_INFO("is_display_origin_depth_image: %d", IS_DISPLAY_ORIGIN_DEPTH_IMAGE);
  n.param<bool>(node_name + "/is_display_resize_depth_image", IS_DISPLAY_RESIZE_DEPTH_IMAGE, true);
  ROS_INFO("is_display_resize_depth_image: %d", IS_DISPLAY_RESIZE_DEPTH_IMAGE);
  n.param<std::string>(node_name + "/camera_info_topic", CAMERA_INFO_TOPIC, "/camera/color/camera_info");
  ROS_INFO("camera_info_topic: %s", CAMERA_INFO_TOPIC.c_str());
  n.param<std::string>(node_name + "/depth_image_topic", DEPTH_IMAGE_TOPIC, "/camera/depth/image_rect_raw");
  ROS_INFO("depth_image_topic: %s", DEPTH_IMAGE_TOPIC.c_str());
  n.param<std::string>(node_name + "/color_image_topic", COLOR_IMAGE_TOPIC, "/camera/color/image_raw");
  ROS_INFO("color_image_topic: %s", COLOR_IMAGE_TOPIC.c_str());

  n.param<double>(node_name + "/width_image_resize", WIDTH_IMAGE_RESIZE, 640);
  ROS_INFO("width_image_resize: %0.0f", WIDTH_IMAGE_RESIZE);
  n.param<double>(node_name + "/height_image_resize", HEIGHT_IMAGE_RESIZE, 480);
  ROS_INFO("height_image_resize: %0.0f", HEIGHT_IMAGE_RESIZE);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "convert_image");
    ros::NodeHandle n;

    std::string node_name = ros::this_node::getName();
    loadParameters(n, node_name);

    depth_image_convert_pub = n.advertise<sensor_msgs::Image>("/depth_image_convert", 10);
    color_image_convert_pub = n.advertise<sensor_msgs::Image>("/color_image_convert", 10);

    ros::Subscriber camera_info_sub = n.subscribe(CAMERA_INFO_TOPIC, 10, cameraInfoCallback);
    ros::Subscriber depth_image_sub = n.subscribe(DEPTH_IMAGE_TOPIC, 10, depthImageCallback);
    ros::Subscriber color_image_sub = n.subscribe(COLOR_IMAGE_TOPIC, 10, colorImageCallback);
    
    ros::spin();
    return 0;
}