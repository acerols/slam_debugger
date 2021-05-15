#include <orb_slam_debugger/orb_slam_debugger.hpp>
#include <cstdio>
#include <iostream>
#include <cv_bridge/cv_bridge.h>

orb_slam_debugger::orb_slam_debugger(
    const std::string &node_name,
    const rclcpp::NodeOptions &node_options
    ) : Node(node_name, node_options), image_num_(0)
{
    declare_parameter("enable_csv", rclcpp::ParameterValue(true));
    declare_parameter("csv_path",   "/home/bkmn/colcon_ws/debug/camera_pose.csv");
    declare_parameter("enable_image", rclcpp::ParameterValue(true));
    declare_parameter("image_path", "/home/bkmn/colcon_ws/image/debug/");
    
}

orb_slam_debugger::~orb_slam_debugger()
{
    
}

void orb_slam_debugger::init()
{
    get_parameter("enable_csv", enable_csv_);
    get_parameter("csv_path", csv_path_);
    get_parameter("enable_image", enable_image_);
    get_parameter("image_path", image_path_);

    ros_clock_ = rclcpp::Clock(RCL_ROS_TIME);
    pose_start_ = image_start_ = ros_clock_.now();

    image_ptr_ = nullptr;

    if(enable_csv_){
        csv_.open(csv_path_);
        if(!csv_.is_open()){
            RCLCPP_ERROR(this->get_logger(), "Could not open csv file");
        }
    }

    debug_camera_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/orb_slam2_stereo_node/pose",
        10,
        std::bind(&orb_slam_debugger::debug_camera_pose_callback_, this, std::placeholders::_1));

    debug_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/orb_slam2_stereo_node/debug_image", 
        rclcpp::QoS(10),
        std::bind(&orb_slam_debugger::debug_image_callback_, this, std::placeholders::_1));

    image_save_ = this->create_wall_timer(std::chrono::milliseconds(16),
                                          std::bind(&orb_slam_debugger::save_image_callback_, this));
}

void orb_slam_debugger::debug_camera_pose_callback_(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
    static bool first = true;
    if(first){
        first = false;
        pose_start_ = pose->header.stamp;
    }
    double start = pose_start_.nanoseconds();

    double dt = pose->header.stamp.nanosec - start;
    char str[256];
    sprintf(str, "%f, %f %f %f %f %f %f %f", dt, pose->pose.position.x, pose->pose.position.y, pose->pose.position.z,
            pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
    csv_ << str << std::endl;

    //RCLCPP_INFO(this->get_logger(), str);

}

void orb_slam_debugger::debug_image_callback_(const sensor_msgs::msg::Image::SharedPtr image)
{
    
    image_ptr_ = image;
    
    image_time_sec_ = int32_t(ros_clock_.now().seconds());
    image_time_nanosec_ = ros_clock_.now().nanoseconds();

    return ;
}

void orb_slam_debugger::save_image_callback_()
{
    static int32_t pre_image_time_sec = 0;
    static int64_t pre_image_time_nanosec = 0;


    RCLCPP_INFO(this->get_logger(), "%d, %d", pre_image_time_nanosec, image_time_nanosec_);
    

    if(image_time_sec_ != pre_image_time_sec 
        || image_time_nanosec_ != pre_image_time_nanosec){

        cv::Mat save_image = cv_bridge::toCvCopy(this->image_ptr_, "bgr8")->image;
        std::string filename = (image_path_ + "debug_image") + std::to_string(image_num_);
        filename = filename + ".jpg";

        RCLCPP_INFO(this->get_logger(), "%s", filename.data());

        cv::imwrite(filename, save_image);

        image_num_++;
        pre_image_time_sec = image_time_sec_;
        pre_image_time_nanosec = image_time_nanosec_;


    }
}