#ifndef __ORB_SLAM_DEBUGGER_H__
#define __ORB_SLAM_DEBUGGER_H__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <string>
#include <iostream>
#include <fstream>

class orb_slam_debugger : public rclcpp::Node{
public:
    orb_slam_debugger(const std::string &node_name="",
        const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
    ~orb_slam_debugger();
    void init();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr debug_camera_pose_sub_;
    std::ofstream csv_;
    rclcpp::Time pose_start_;
    rclcpp::Time image_start_;
    rclcpp::Clock ros_clock_;
    rclcpp::TimerBase::SharedPtr image_save_;
    sensor_msgs::msg::Image::SharedPtr image_ptr_;
    bool enable_csv_;
    bool enable_image_;
    std::string csv_path_;
    std::string image_path_;
    void debug_image_callback_(const sensor_msgs::msg::Image::SharedPtr image);
    void debug_camera_pose_callback_(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
    void save_image_callback_();
    uint64_t image_num_;
    
    cv::Mat save_image_;
    int32_t image_time_sec_;
    int64_t image_time_nanosec_;

};

#endif