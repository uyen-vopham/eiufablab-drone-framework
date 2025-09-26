#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>




#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


#include "std_srvs/srv/set_bool.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/waypoint_pull.hpp"
#include "custom_msgs/srv/mode_signal.hpp"

// #include "offboard_control/offboard_control.hpp"



using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node 
{
public:
    OffboardControl();
    

private:
    // Callbacks
    void process_wp();
    void process_wp_cb(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future);
    void arm_drone();
    void arm_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future);
    void state_cb(const mavros_msgs::msg::State msg);
    void set_offboard_mode();
    void set_mode_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
    void main_loop();
    void position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void takeoff();
    void go_ahead();
    void waypoint_cb(const mavros_msgs::msg::WaypointList::SharedPtr msg);
    void log_position();
    void disarm();
    void disarm_cb(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future);
    void landing();
    void landing_cb(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
    void service_callback(const std::shared_ptr<custom_msgs::srv::ModeSignal::Request> request,
          std::shared_ptr<custom_msgs::srv::ModeSignal::Response>      response);
    // void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    //       std::shared_ptr<std_srvs::srv::SetBool::Response>      response);
    void follow_trajectory(const std::string& csv_to_read_path);
    void pull_waypoint();
    void pull_waypoint_cb(rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedFuture future);
    
    // void hover_time(rclcpp::Time start_time_, double period);
    // bool is_stable_position();


    // Members
    mavros_msgs::msg::State current_state_;
    double current_x_, current_y_, current_z_, takeoff_height_;
    double current_lat_, current_long_, current_alt_;
    bool takeoff_flag, go_ahead_flag, offboard_flag=true, hover_flag=false;
    bool check_armed_, check_landed_, reach_attitude_;
    bool landing_flag_, offboard_mode_;
    bool landing_started_, delay_started_;
    bool pull_waypoint_srv_flag;
    bool send_takeoff_;
    bool process_srv_flag;
    

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_sub_;
    rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr waypoint_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture shared_future;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr landing_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr process_wp_client_;
    rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedPtr pull_waypoint_client_;
    rclcpp::Service<custom_msgs::srv::ModeSignal>::SharedPtr service_;
    // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::SubscriptionOptions subscription_options_;
    rclcpp::TimerBase::SharedPtr timer_, timer_csv;
    rclcpp::Time hover_start_time, csv_start_, hover_stand_start_time, takeoff_delay_start_time_;
    rclcpp::Time last_pose_time_, last_state_time_, attitude_reach_time_, takeoff_start_time_;
    rclcpp::Duration takeoff_delay_;
    std::ofstream csv_file_;


    geometry_msgs::msg::PoseStamped pose_, target_pose, current_pose_;
    mavros_msgs::msg::WaypointList waypoint_list;
    // mavros_msgs::srv::CommandBool disarm_client;
    rclcpp::Time last_request_;
   
};
