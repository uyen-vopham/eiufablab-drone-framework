#include "offboard_control/offboard_control.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
#include <functional> 

OffboardControl::OffboardControl(): Node("offboard_control_server"),                                           
                                                              check_armed_(false),
                                                              reach_attitude_(false),
                                                              landing_flag_(false),
                                                              offboard_mode_(false),
                                                              pull_waypoint_srv_flag(false),
                                                              process_srv_flag(false),
                                                              takeoff_start_time_(this->now()),
                                                              takeoff_delay_(rclcpp::Duration::from_seconds(5.0))
{
    //------------Initial variables----------------
    delay_started_ = false;
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = takeoff_height_;
    //------------Parameter------------------------
    this->declare_parameter("takeoff_height_", 7.0);
    this->get_parameter("takeoff_height_", takeoff_height_);

    //-------------QoS setting for different topics-------
    //----DRONE STATE
    rclcpp::QoS qos_state(rclcpp::KeepLast(10));
    qos_state.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_state.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    //----DRONE POSE
    rclcpp::QoS qos_pose(rclcpp::KeepLast(10));
    qos_pose.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_pose.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    //----DRONE PUBLISH
    rclcpp::QoS qos_pub(rclcpp::KeepLast(10));
    qos_pub.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_pub.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    //----DRONE BATTERY
    rclcpp::QoS qos_battery(rclcpp::KeepLast(10));
    qos_battery.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_battery.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    //----DRONE WAYPOINTS  
    rclcpp::QoS qos_waypoint(rclcpp::KeepLast(10));
    qos_waypoint.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_waypoint.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    //QoS for subriber (example: LIDAR, pose,..)
   
    
    //------------Reentrant---------------
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);



    //------------Subscriber--------------
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", qos_state,
       std::bind(&OffboardControl::state_cb, this, _1)
    );
    local_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", qos_pose, 
        std::bind(&OffboardControl::position_cb, this, _1)
    );
    // waypoint_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
    //     "offboard_waypoint_list", qos_waypoint,
    //     std::bind(&OffboardControl::waypoint_cb, this, _1)
    // );

    //-------------Publisher------------
    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "mavros/setpoint_position/local", qos_pub
    );


    //---------------Service Server--------------
    // service_ = this-> create_service <std_srvs::srv::SetBool>("offboard_service", std::bind(&OffboardControl::service_callback, this, _1, _2), rmw_qos_profile_services_default,callback_group_);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to send response");
    service_ = this-> create_service <drone_msgs::srv::ModeSignal>("offboard_service", std::bind(&OffboardControl::service_callback, this, _1, _2), rmw_qos_profile_services_default,callback_group_);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to send response");


  
    //---------------Service client--------------
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    //check if the service is available
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    //landing client
    landing_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    //process waypoint client
    process_wp_client_ = this->create_client<std_srvs::srv::SetBool>("process_waypoint_service");

    
    //--------------------Timer callback----------------
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&OffboardControl::main_loop, this)
    );

    //--------------Initial other functions-----------------
    timer_csv = this->create_wall_timer(std::chrono::milliseconds(100), 
                                        std::bind(&OffboardControl::log_position, this));
    csv_start_ = this->now();
}


void OffboardControl::service_callback(const std::shared_ptr<drone_msgs::srv::ModeSignal::Request> request,
          std::shared_ptr<drone_msgs::srv::ModeSignal::Response>      response)
{
    if (request->mode == drone_msgs::srv::ModeSignal::Request::OFFBOARD)
    {response -> success = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response ");
    takeoff_flag = true;
    takeoff_start_time_ = this->now();
    offboard_mode_ = false;
    }
    else
    {RCLCPP_INFO(this->get_logger(), "❗REQUEST RECEIVE, BUT NOT OFFBOARD MODE");}
}

// void OffboardControl::waypoint_cb(const mavros_msgs::msg::WaypointList::SharedPtr msg){
//     waypoint_list = *msg;
//     RCLCPP_INFO(this->get_logger(), "WAYPOINT LIST RECEIVED", waypoint_list.waypoints);
// }

void OffboardControl::set_offboard_mode() {
    if (offboard_mode_ || current_state_.mode == "OFFBOARD") {
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD mode...");
    if (!set_mode_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Set mode service not available");
        return;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request-> custom_mode = "OFFBOARD";

    auto future = set_mode_client_->async_send_request(request,
        std::bind(&OffboardControl::set_mode_callback, this, std::placeholders::_1));
}

void OffboardControl::set_mode_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->mode_sent) {
            RCLCPP_INFO(this->get_logger(), "OFFBOARD mode set successfully!");
            offboard_mode_ = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to set OFFBOARD mode");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
}

void OffboardControl::state_cb(const mavros_msgs::msg::State msg) {
    current_state_ = msg;  //SharedPtr là con trỏ, current_state_ là 1 biến bình thường, và đã có địa chỉ
                            //Để gán giá trị msg vào biến current_state_, thì phải thêm * vào msg để lấy giá trị của biến địa chỉ msg
    last_state_time_=this->now();
    if (current_state_.mode == "OFFBOARD" && offboard_flag)
    {
        // arm_drone();
        RCLCPP_INFO(this->get_logger(), "OFFBOARD MODE");
        // takeoff_flag = true;
        // RCLCPP_INFO(this->get_logger(), "CURRENT Z IS %f", current_z_);
        landing_flag_=false;
    }
    if (current_state_.mode == "AUTO.LAND"){
        landing_flag_ = true;
        offboard_flag = false;
        RCLCPP_INFO(this->get_logger(), "AUTO.LAND MODE");
    }
    
    if (current_state_.mode == "STABILIZED"){
        offboard_flag = true;
    }
}

void OffboardControl::process_wp (){
    if (process_srv_flag){
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Requesting to process waypoints...");
    if (!process_wp_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Process waypoints service not available");
        return;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto future = process_wp_client_->async_send_request(request,
        std::bind(&OffboardControl::process_wp_cb, this, std::placeholders::_1));
    
    process_srv_flag = true;
}

void OffboardControl::process_wp_cb(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future){
    try
    {
        auto response_process_wq = future.get();

        if (response_process_wq->success)
        {
            RCLCPP_INFO(this->get_logger(), "✅ Recieving transfer waypoints successfully.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "❌ PROCESS WAYPOINTS request sent but failed.");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Error receiving PROCESS WAYPOINTS response: %s", e.what());
    }
}

void OffboardControl::position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg){ 
    last_pose_time_ = this->now();
    current_x_ = msg->pose.position.x;
    current_y_ = msg->pose.position.y;
    current_z_ = msg->pose.position.z;
    current_pose_ = *msg;

    if (!reach_attitude_ && current_z_ >= takeoff_height_ -0.1){
        reach_attitude_ = true;
        attitude_reach_time_=this->now();
        RCLCPP_INFO(this->get_logger(), "✅ Target altitude reached, starting countdown...");
    }
}

void OffboardControl::arm_drone(){
    if (current_state_.armed){
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Requesting to arm the drone...");
    if (!arming_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Arming service not available");
        return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    auto future = arming_client_->async_send_request(request,
        std::bind(&OffboardControl::arm_callback, this, std::placeholders::_1));
}

void OffboardControl::arm_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future){
    try
    {
        auto response = future.get();

        if (response->success){
            RCLCPP_INFO(this->get_logger(), "Drone Armed!");
            check_armed_=true;
            // pull_waypoint_srv_flag = true;
            // pull_waypoint();
        }
        else  {RCLCPP_WARN(this->get_logger(), "Failed to Arm Drone!");}

    }catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    } 
}

void OffboardControl::disarm(){
    if (!current_state_.armed){
        RCLCPP_INFO(this->get_logger(), "❎ Drone is already DISARMED, no need to send request.");
        return; 
    }
    RCLCPP_INFO(this->get_logger(), "🟠 Sending DISARM request...");
    auto disarm_client_ = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    disarm_client_->value = false;
    if (!arming_client_->wait_for_service(std::chrono::seconds(2))){
        RCLCPP_WARN(this->get_logger(), "Arming client not available");
        return;
    }

    auto future = arming_client_->async_send_request(disarm_client_,
        std::bind(&OffboardControl::disarm_cb, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Disarm the drone");
}

void OffboardControl::disarm_cb(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future){
    try
    {
        auto response_arm = future.get();

        if (response_arm->success)
        {
            RCLCPP_INFO(this->get_logger(), "✅ Drone DISARMED successfully.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "❌ DISARM request sent but failed.");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Error receiving DISARM response: %s", e.what());
    }
}

void OffboardControl::go_ahead(){
    target_pose.header.stamp = this->now(); 
    target_pose.header.frame_id = "map";
    target_pose.pose.position.x = 25.0;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 7.0;
    // setpoint_pub_->publish(target_pose);
}

void OffboardControl::takeoff(){
    target_pose.header.stamp = this->now();
    target_pose.header.frame_id = "map";
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.position.z = takeoff_height_;
    setpoint_pub_->publish(target_pose);
}

void OffboardControl::landing(){
    if (landing_flag_){return;}
    RCLCPP_INFO(this->get_logger(), "Requesting to arm the drone...");
    if (!landing_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Landing service not available");
        return;
    }
    auto request_landing = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request_landing->base_mode = 0;
    request_landing-> custom_mode = "AUTO.LAND";
    // landing_automode = true;

    auto landing_future = landing_client_->async_send_request(request_landing,
    std::bind(&OffboardControl::landing_cb, this, std::placeholders::_1));
    landing_flag_ = true;
}

void OffboardControl::landing_cb(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future){
    try
    {
        auto response = future.get();

        if (response->mode_sent){
            RCLCPP_INFO(this->get_logger(), "AUTO.LAND mode sent successfully!");
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Failed to set AUTO.LAND mode");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
}

void OffboardControl::main_loop() {

   // If this is the first time the timer runs → start the delay countdown
    if (!delay_started_){
        takeoff_delay_start_time_ = this->now();
        delay_started_ = true;
        RCLCPP_INFO(this->get_logger(), "⏳ Starting takeoff delay (%.1fs)...", takeoff_delay_.seconds());
        return;
    }
    // If delay time has not passed yet → only publish the target pose
    if ((this->now() - takeoff_delay_start_time_)< takeoff_delay_){
        setpoint_pub_->publish(target_pose);
        return;   
    }
    // Delay completed → proceed with normal flight operations
    if ((this->now()-last_pose_time_).seconds()>1.0 || (this->now()-last_state_time_).seconds()>1.5){
        RCLCPP_WARN(this->get_logger(), "⚠️ No pose or state received in the last 1 second!");
        return;
    }
    // Check lost connection to FC
    if (!current_state_.connected){
        RCLCPP_ERROR(this->get_logger(), "❌ Lost connection to FCU! Stopping setpoint publication.");
        return;
    }

    /////////////////////////////////////

    if (!landing_flag_ && takeoff_flag){
        process_wp();
        set_offboard_mode();
        // if (current_state_.mode == "OFFBOARD" && offboard_mode_) {
        // RCLCPP_INFO(this->get_logger(), "Waiting for OFFBOARD mode...");
        // return;
        // }   
        arm_drone();
        // pull_waypoint();
        takeoff();
    }

    if (!reach_attitude_ && (this->now() - takeoff_start_time_) > rclcpp::Duration(30s)){
        RCLCPP_ERROR(this->get_logger(), "🕒 Timeout: Failed to reach target altitude within 30 seconds after ARMED. Initiating landing...");
        this->landing();
        return;
    }



    if (check_armed_ && reach_attitude_ && (this->now() - attitude_reach_time_) > 10s && !landing_flag_){
        landing();
        RCLCPP_INFO(this->get_logger(), "🔻 Landing initiated after holding altitude for 10 seconds");
        return;
    }

    if (landing_flag_ && !current_state_.armed){
        check_landed_ = true;
        RCLCPP_INFO(this->get_logger(), "✅ Drone has successfully landed.");

        if (timer_){
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "⏹️ Timer has been stopped to save CPU resources.");
        }
    }
}

void OffboardControl::log_position(){
    double elapsed_time = (this->now() - csv_start_).seconds();
    try
    {
        std::ofstream file("drone_log_position.csv", std::ios::app);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", "drone_log_position.csv" );
            return;
        }

        file << std::fixed << std::setprecision(2)
             << elapsed_time << ","
             << current_x_ << ","
             << current_y_ << ","
             << current_z_ << "\n";
        file.close();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to CSV: %s", e.what());
    }
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControl>();

    // Instead of spin, use a drone executor
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
