/**
 * @file image_saver.cpp
 * @author Zhihao Zhan (zhanzhihao_dt@163.com)
 * @brief save image from camera to local disk
 * @version 0.1
 * @date 2025-09-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <iostream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

int image_count = 0;
ros::Subscriber sub;

void ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
  const char *user_name = getlogin();
  std::string filename = "/home/" + std::string(user_name) + "/image_" +
                         std::to_string(msg->header.stamp.toNSec()) + ".png";
  cv::imwrite(filename, img);
  ROS_INFO_STREAM("Saved image to " << filename);
  image_count++;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_saver");
  ros::NodeHandle nh;
  sub = nh.subscribe("/left_camera/image", 1, ImageCallback);
  ROS_INFO("Image saver node started, waiting for images...");

  while (ros::ok()) {
    if (image_count >= 1) {
      ROS_INFO("Image saved, break loop.");
      break;
    }

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}