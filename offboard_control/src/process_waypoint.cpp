#include "offboard_control/process_waypoint.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
#include <functional> 

ProcessWaypointNode :: ProcessWaypointNode (): Node("process_waypoint"), recieved_rq(false),
                                                                        response_sig(false),
                                                                        write_transfer_wp_flag(false)
{
    
    //------------Declare parameters------
    this->declare_parameter<std::string>("csv_to_write_waypoint");
    this->declare_parameter<std::string>("csv_to_write_transfer_waypoint");

    //------------Get parameters----------
    this -> csv_to_write_waypoint = this->get_parameter("csv_to_write_waypoint").as_string();
    this -> csv_to_write_transfer_waypoint = this->get_parameter("csv_to_write_transfer_waypoint").as_string();
    csv_to_read_waypoint = csv_to_write_waypoint;
    //------------QoS---------------------------
    //----DRONE WAYPOINTS  
    rclcpp::QoS qos_waypoint(rclcpp::KeepLast(10));
    qos_waypoint.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_waypoint.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    //----DRONE POSITION  
    rclcpp::QoS qos_position_global(rclcpp::KeepLast(10));
    qos_position_global.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_position_global.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    //------------Reentrant---------------
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    //---------------Service Server--------------
    service_ = this-> create_service <std_srvs::srv::SetBool>("process_waypoint_service", 
                    std::bind(&ProcessWaypointNode::service_callback, this, _1, _2)
                    , rmw_qos_profile_services_default,callback_group_);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to send response");

    //---------------Service Client--------------
    //pull waypoin client
    // pull_waypoint_client_ = this ->create_client<mavros_msgs::srv::WaypointPull>("mavros/mission/pull");


    //---------------Suscribe topics-------------
    // waypoint_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
    //     "/offboard_waypoint_list", qos_waypoint,
    //     std::bind(&ProcessWaypointNode::waypoint_cb, this, _1)
    // );

    org_position_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "mavros/global_position/global",
      rclcpp::SensorDataQoS(),  // pick QoS you need
      [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (got_first_.exchange(true)) return;   // ignore if already handled
        RCLCPP_INFO(this->get_logger(), "Got first fix: lat=%.7f lon=%.7f",
                    msg->latitude, msg->longitude);
        org_lat_ = msg->latitude;
        org_long_ = msg->longitude;
        org_alt_ = 0.0;
        // do whatever you need with the first message...
        // then unsubscribe:
        org_position_.reset();  // stops receiving further messages
        RCLCPP_INFO(this->get_logger(), "Unsubscribed after first message.");
      }
    );

    //--------------------Timer callback----------------
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ProcessWaypointNode::main_loop, this)
    );


    //---------------------Other intializtions----------
    std::ofstream file(csv_to_write_transfer_waypoint, std::ios::trunc);

}

void ProcessWaypointNode::service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>  response)
{
    recieved_rq = true;
    // response -> success = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "🍊 Incoming PROCESS WAYPOINT request");
    waypoint_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
        "/offboard_waypoint_list", this->qos_waypoint,
        std::bind(&ProcessWaypointNode::waypoint_cb, this, _1)
    );
    if (write_transfer_wp_flag)
    {response -> success = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response from process_waypoint ");}
    else {
        // có thể reset để ngưng sub
        waypoint_sub_.reset();
        response->success = true;
        response->message = "Subscription deactivated";
    }
    

    // takeoff_flag = true;
}

// void ProcessWaypointNode::pull_waypoint(){
//     if (pull_waypoint_srv_flag){
//         // RCLCPP_INFO(this->get_logger(), "❎ Drone is already send client to pull waypoint.");
//         return; 
//     }
    
//     if (!pull_waypoint_client_->wait_for_service(std::chrono::seconds(2))){
//         RCLCPP_WARN(this->get_logger(), "Waypoint pull service not available");
//         return;
//     }
//     auto pull_request_ = std::make_shared<mavros_msgs::srv::WaypointPull::Request>();
//     auto future = pull_waypoint_client_->async_send_request(pull_request_,
//         std::bind(&ProcessWaypointNode::pull_waypoint_cb, this, std::placeholders::_1));
//     RCLCPP_INFO(this->get_logger(), "PULLED");
//     pull_waypoint_srv_flag = true;
// }

// void ProcessWaypointNode::pull_waypoint_cb(rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedFuture future){
//     try
//     {
//         auto response = future.get();
//         if (response->success)
//         {
//             RCLCPP_INFO(this->get_logger(),
//                 "Successfully pulled %u waypoints from FCU", response->wp_received);
//         }
//         else
//         {
//             RCLCPP_WARN(this->get_logger(), "Failed to pull mission waypoints from FCU");
//         }
//     }
//     catch (const std::exception &e)
//     {
//         RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
//     }
// }


void ProcessWaypointNode::waypoint_cb(const mavros_msgs::msg::WaypointList::SharedPtr msg){
    // RCLCPP_INFO(this->get_logger(), "Received %ld waypoints", msg->waypoints.size());
    RCLCPP_INFO(this->get_logger(), "YOU ARE IN WAYPOINT CALLBACK");
    RCLCPP_INFO(this->get_logger(),
        "Got %zu waypoints, current waypoint seq: %u",
        msg->waypoints.size(), msg->current_seq);

    // RCLCPP_INFO(this->get_logger(),  "WP are %.9d", msg->waypoints);
    std::cout<<csv_to_write_waypoint<<std::endl;
    std::ofstream file (csv_to_write_waypoint, std::ios::trunc);
    if (!file.is_open()){
        RCLCPP_ERROR(this->get_logger(), "❌ Cannot open waypoints_log.csv");
        return;
    }
    if (file.is_open()){RCLCPP_INFO(this->get_logger(), "FILE IS OPENEDDDDDDDDDDDDDDDD");}
    // std::string name = "uyen";
    // int age = 29;
    // file << name << "," << age << "\n";
    for (size_t i = 0; i < msg->waypoints.size(); i++)
    {
        const auto &wp = msg->waypoints[i];
        RCLCPP_INFO(this->get_logger(),
            "WP[%zu] frame=%d command=%d is_current=%d "
            "lat=%.9f lon=%.9f alt=%.9f",
            i,
            wp.frame,
            wp.command,
            wp.is_current,
            wp.x_lat,
            wp.y_long,
            wp.z_alt
        );
        file << std::fixed << std::setprecision(10)
            << wp.x_lat << ","
            << wp.y_long << ","
            << wp.z_alt << "\n";
    }
    file.close();
    std::cout << "✅ CSV written successfully.\n";
    return;
}

std::string ProcessWaypointNode::convertGPS2ENU_function (double lat0, double long0, double alt0, 
                std::vector<double> line){
    //  double lat0 = 11.052945, lon0 = 106.6661470, alt0 = 0;     // gốc local
    // double lat = 11.0529021, lon = 106.6662267, alt = 7.0;     // target GPS

    GeographicLib::LocalCartesian proj(lat0, long0, alt0);  // khởi tạo hệ ENU

    double x, y, z;
    proj.Forward(line[0], line[1], line[2], x, y, z);   // GPS → ENU

    // std::cout << "x (East): " << x << ", y (North): " << y << ", z (Up): " << z << std::endl;
    std::stringstream oss;
    oss << std::fixed << std::setprecision(7)
    << x << "," << y << "," << z;
    return oss.str();
}

void ProcessWaypointNode::read_csv_file (const std::string& csv_to_read_path, const std::string& csv_to_write_path,
    const float& lat0, const float& long0, const float& alt0)
{
    if (write_transfer_wp_flag){return;}
    std::ifstream file (csv_to_read_path);
    std::cout<<csv_to_read_path<<std::endl;
    if (!file.is_open()){
        std::cout<<"ERROR: Can not open csv file"<< std::endl;
        // RCLCPP_ERROR(this->get_logger(), "❌ Cannot open waypoints_log.csv");
        return;
    }
    if (file.is_open()){
        std::cout<<"INFO: CSV file is open, and ready to transfer."<<std::endl;}
    
    std::string line;
    while(std::getline(file, line))
    {
        std::stringstream single_stream_(line);
        std::string token;
        // std::getline(single_stream_, token, ',');  // đọc và bỏ token đầu (index)
        std::vector<double> values;
        while(std::getline(single_stream_, token, ','))
        {
            values.push_back(std::stod(token));
            // std::cout << token << std::endl;
        }
        auto convert_values = convertGPS2ENU_function(lat0, long0, alt0, values);
        std::cout << convert_values << std::endl;
        // std::cout << values[1] << std::endl;
        // std::cout << values[2] << std::endl;
        write_csv_file(csv_to_write_path, convert_values);
        
        std::cout<<std::endl;
    }   
    file.close();
    write_transfer_wp_flag=true;
}

void ProcessWaypointNode::write_csv_file(const std::string& csv_path_to_write, std::string& line)
{
    //  std::cout << line << std::endl;
    // int order;
    // float X, Y, Z;
    // order = line[0];
    // X = line[1];
    // Y = line[2];
    // Z = line[3];
  
    std::ofstream file (csv_path_to_write, std::ios::app);
    if (!file.is_open()){
        std::cout<<"ERROR: Can not open convert_waypoint_log.csv file"<<std::endl;
        return;
    }
    if (file.is_open()){std::cout<<"INFO: CSV file is open, and ready to write."<<std::endl;}
    // file <<"Order, Latitude, Longitude, Altitude\n";
    file << line << "\n";
    file.close();
    std::cout << "✅ CSV written successfully.\n";
    return;
}

void ProcessWaypointNode::main_loop() {
    if (recieved_rq)
    {
    // pull_waypoint();
    read_csv_file(csv_to_read_waypoint, csv_to_write_transfer_waypoint, org_lat_, org_long_, org_alt_);}
    else {return;}
   
    
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProcessWaypointNode>();

    // Instead of spin, use a custom executor
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}