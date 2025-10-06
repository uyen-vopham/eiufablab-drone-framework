#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"


using namespace std::chrono_literals;

class DroneTransformation : public rclcpp::Node 
{
    public:
    DroneTransformation();
    void position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    // void subscribe_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void main_loop();



    geometry_msgs::msg::TransformStamped static_transform_;
    geometry_msgs::msg::TransformStamped dynamic_transform_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;

    private:
    //-----Inherrit------
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_sub_;
    double last_x, last_y, last_z;
    double x_increment, y_increment, z_increment;
    rclcpp::TimerBase::SharedPtr timer_;
};

