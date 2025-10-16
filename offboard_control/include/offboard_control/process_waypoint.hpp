#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <atomic>



#include "python3.10/Python.h"


#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/waypoint_pull.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "GeographicLib/LocalCartesian.hpp"

//#include "/home/uyen/Drone/drone_ws/src/offboard_control/include/offboard_control/offboard_control.hpp"

#include "offboard_control/offboard_control.hpp"


using namespace std::chrono_literals;

class ProcessWaypointNode: public rclcpp::Node 
{
    public:
    ProcessWaypointNode();
    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>  response);
    void read_csv_file (const std::string& csv_to_read_path, const std::string& csv_to_write_path, const float& lat0, 
                        const float& long0, const float& alt0);
    std::string convertGPS2ENU_function(double lat0, double long0, double alt0, std::vector<double> line);
    void write_csv_file(const std::string& csv_path_to_write, std::string& line);
    void pull_waypoint(const std::string& csv_to_read_path);
    void pull_waypoint_cb(rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedFuture future);
    void waypoint_cb(const mavros_msgs::msg::WaypointList::SharedPtr msg);
    void minimum_snap_python();
    void main_loop();


    private:
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedPtr pull_waypoint_client_;
    rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr waypoint_sub_org_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr org_position_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr waypoint_processed_pub_;
    
    mavros_msgs::msg::PositionTarget target;
    
    bool recieved_rq, response_sig;
    bool pull_waypoint_srv_flag, write_transfer_wp_flag;
    bool get_org_pos;
    bool transfered_wp_flag = false;
    double org_lat_, org_long_, org_alt_;
    bool optimize_flag;

    std::atomic<bool> got_first_;
    std::string csv_to_write_waypoint;
    std::string csv_to_read_waypoint;
    std::string csv_to_write_transfer_waypoint;
    std::string csv_to_optimize_waypoint;

    //Python declaration
    PyObject* drone_raw_time;
    PyObject* drone_raw_pos;
    PyObject* drone_raw_vel;
    PyObject* drone_raw_acc;

    // std::tuple<double> drone_raw_time, drone_raw_pos, drone_raw_vel, drone_raw_acc;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::QoS qos_waypoint{rclcpp::KeepLast(10)};
    
};