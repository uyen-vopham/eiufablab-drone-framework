#include "offboard_control/process_waypoint.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
#include <functional> 

ProcessWaypointNode :: ProcessWaypointNode (): Node("process_waypoint"), recieved_rq(false),
                                                                        response_sig(false),
                                                                        write_transfer_wp_flag(false),
                                                                        transfered_wp_flag(false)
                                                                        
{
    
    //------------Declare parameters------
    this->declare_parameter<std::string>("csv_to_write_waypoint");
    this->declare_parameter<std::string>("csv_to_write_transfer_waypoint");
    this->declare_parameter<std::string>("csv_to_read_waypoint");

    //------------Get parameters----------
    this -> csv_to_write_waypoint = this->get_parameter("csv_to_write_waypoint").as_string();
    this -> csv_to_write_transfer_waypoint = this->get_parameter("csv_to_write_transfer_waypoint").as_string();
    this -> csv_to_read_waypoint = this->get_parameter("csv_to_read_waypoint").as_string();
    // csv_to_read_waypoint = csv_to_write_waypoint;
    // csv_to_read_waypoint = "/home/uyen/Drone/drone_ws/src/offboard_control/src/csv_files/minimum_snap_traj.csv";
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


   //-----------------Publish topics--------------
    waypoint_processed_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
        "/offboard_control/waypoint_processed", qos_waypoint
    );
   
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
    target.header.stamp = this->now();
    target.header.frame_id = "map";
    target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

    //Bỏ qua vận tốc, gia tốc 
    target.type_mask = (mavros_msgs::msg::PositionTarget::IGNORE_VX |
                       mavros_msgs::msg::PositionTarget::IGNORE_VY |
                       mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                       mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                       mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                       mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                       mavros_msgs::msg::PositionTarget::FORCE);

}



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

void ProcessWaypointNode::pull_waypoint(const std::string& csv_to_read_path)
{
    if (transfered_wp_flag) { return; }

    std::ifstream file(csv_to_read_path);
    std::cout << csv_to_read_path << std::endl;

    if (!file.is_open()) {
        std::cout << "ERROR: Cannot open csv file" << std::endl;
        return;
    }

    std::cout << "INFO: CSV file is open, and ready to transfer." << std::endl;

    // ✅ Preview file content
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    std::cout << "🧾 File content preview:\n" << content.substr(0, 200) << std::endl;

    // 👉 Reset lại luồng để đọc lại từ đầu
    file.clear();  // clear EOF flag
    file.seekg(0, std::ios::beg);

    std::string line;
    int line_count = 0;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        line_count++;

        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;

        while (std::getline(ss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            } catch (...) {
                // có thể dòng đầu là header, bỏ qua
            }
        }
    }

    std::cout << "📄 Tổng số hàng (waypoints) trong CSV: " << line_count << std::endl;
    transfered_wp_flag = true;
    
}

// create a function that used for call python script that calculate minimum snap trajectory for optimizing waypoints
void ProcessWaypointNode::minimum_snap(){
    Py_Initialize(); //Initial Python interpreter
    // Add path to folder that contains file WaypointOptimization.py
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/home/uyen/Drone/drone_ws/src/offboard_control/python_script/Mathematic')");

    //Import module OptimizeWaypoints (class OptimizeWaypoints)
    minimum_snap_name = PyUnicode_DecodeFSDefault("WaypointOptimization");
    minimum_snap_module = PyImport_Import(minimum_snap_name);
    Py_DECREF(minimum_snap_name);

    if (minimum_snap_module != nullptr){
        // Take class OptimzeWaypoints in module
        minimum_snap_class = PyObject_GetAttrString(minimum_snap_module, "OptimizeWaypoints");
        if (minimum_snap_class && PyCallable_Check(minimum_snap_class)){
            minimum_snap_instance = PyObject_CallObject(minimum_snap_class, nullptr);

            //Call generate_trajectorty_from_csv
            minimum_snap_generate = PyObject_GetAttrString(minimum_snap_instance, "generate_trajectory_from_csv");

            if (minimum_snap_generate && PyCallable_Check(minimum_snap_generate)){
                minimum_snap_args = Py_BuildValue("(s)", csv_to_write_transfer_waypoint);
                minimum_snap_result = PyObject_CallObject(minimum_snap_generate, minimum_snap_args);
            
                if (minimum_snap_result){
                    std::cout << "✅ Trajectory generated successfully.\n";
                    Py_DECREF(minimum_snap_result);
                }
                else{
                    PyErr_Print();
                    std::cerr << "❌ Error calling generate_trajectory_from_csv()\n";
                }
                Py_XDECREF(minimum_snap_args);
                Py_XDECREF(minimum_snap_result);
                drone_raw_time = PyTuple_GetItem(minimum_snap_result, 0);
                drone_raw_pos = PyTuple_GetItem(minimum_snap_result, 1);
                drone_raw_vel = PyTuple_GetItem(minimum_snap_result, 2);
                drone_raw_acc = PyTuple_GetItem(minimum_snap_result, 3);
            }
            // PyObject* drone_raw_time = PyTuple_GetItem(minimum_snap_result, 0);
            //Call check_dynamic_limits
            minimum_snap_check_dynamic = PyObject_GetAttrString(minimum_snap_instance, "check_dynamic_limits");
            if (minimum_snap_check_dynamic && PyCallable_Check(minimum_snap_check_dynamic)){
                args_check = Py_BuildValue("(00)", drone_raw_vel, drone_raw_acc);
                result_check = PyObject_CallObject(minimum_snap_check_dynamic, args_check);

                if (result_check){
                    bool in_limit = PyObject_IsTrue(result_check);
                    // Py_DECREF(pCheckResult);
                }
                else{
                    PyErr_Print();
                    std::cerr << "❌ Error calling check_dynamic_limits()\n";
                }
                Py_XDECREF(args_check);
                Py_XDECREF(result_check);
            }
        Py_XDECREF(minimum_snap_instance); 
        Py_XDECREF(minimum_snap_class);
        Py_XDECREF(minimum_snap_generate);
        Py_XDECREF(minimum_snap_check_dynamic);
        }
            
    }
        // 🔚 Dọn dẹp
    Py_XDECREF(minimum_snap_module);
    Py_XDECREF(minimum_snap_name);

    Py_Finalize();
}

void ProcessWaypointNode::main_loop() {
    if (recieved_rq)
    {
    pull_waypoint(csv_to_read_waypoint);
    // read_csv_file(csv_to_read_waypoint, csv_to_write_transfer_waypoint, org_lat_, org_long_, org_alt_);
    }
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