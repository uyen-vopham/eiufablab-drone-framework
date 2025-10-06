#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>







#include "rclcpp/rclcpp.hpp"
// #include "tf2_ros/trans"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
// #include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"



using namespace std::chrono_literals;

class MarkerPublish : public rclcpp::Node 
{
    public: 
        MarkerPublish();
        std::vector<geometry_msgs::msg::Point> read_csv_(const std::string& csv_path);

    private:
        //callback functions
        void publish_points_(const std::vector<geometry_msgs::msg::Point>& points);
        void publish_path_(const std::vector<geometry_msgs::msg::Point>& points);
        void main_loop_();
        
        


        //members
        rclcpp::TimerBase::SharedPtr timer_;
        visualization_msgs::msg::Marker marker_;
        visualization_msgs::msg::MarkerArray marker_array_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr point_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>:: SharedPtr path_pub_;
        // std::vector<std::array<float, 3>> points;
        std::vector<geometry_msgs::msg::Point> point_;

        bool published_flag_;
        std::string csv_path_;
        
};