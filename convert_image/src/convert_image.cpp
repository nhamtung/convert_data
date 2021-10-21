#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <iostream>

ros::Publisher depth_image_convert_pub, color_image_convert_pub;
std::string DEPTH_IMAGE_TOPIC, COLOR_IMAGE_TOPIC;

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
    
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat depth_colormap;
    cv::convertScaleAbs(cv_ptr->image, depth_colormap, 0.03);
    cv::applyColorMap(depth_colormap, depth_colormap, cv::COLORMAP_JET);
    cv::imshow("convert_depth_image", depth_colormap);
    cv::waitKey(10);
}

void loadParameters(ros::NodeHandle n, std::string node_name){
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

    std::string node_name = ros::this_node::getName();
    loadParameters(n, node_name);

    depth_image_convert_pub = n.advertise<sensor_msgs::Image>("/depth_image_convert", 10);
    color_image_convert_pub = n.advertise<sensor_msgs::Image>("/color_image_convert", 10);

    ros::Subscriber depth_image_sub = n.subscribe(DEPTH_IMAGE_TOPIC, 10, depthImageCallback);
    ros::Subscriber color_image_sub = n.subscribe(COLOR_IMAGE_TOPIC, 10, colorImageCallback);
    
    ros::spin();
    cv::destroyWindow("convert_color_image");
    cv::destroyWindow("convert_depth_image");
    return 0;
}