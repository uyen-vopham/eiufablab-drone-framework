#include "offboard_control/process_waypoint.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
#include <functional> 

ProcessWaypointNode :: ProcessWaypointNode (): Node("process_waypoint"), recieved_rq(false),
                                                                        response_sig(false),
                                                                        write_transfer_wp_flag(false),
                                                                        transfered_wp_flag(false),
                                                                        recieve_wp(false),
                                                                        optimize_flag(false)
                                                                        
{
    
    //------------Declare parameters------
    this->declare_parameter<std::string>("csv_to_write_waypoint");
    this->declare_parameter<std::string>("csv_to_write_transfer_waypoint");
    // this->declare_parameter<std::string>("csv_to_read_waypoint");
    // this->declare_parameter<std::string>("csv_to_optimize_waypoint");

    //------------Get parameters----------
    this -> csv_to_write_waypoint = this->get_parameter("csv_to_write_waypoint").as_string();
    this -> csv_to_write_transfer_waypoint = this->get_parameter("csv_to_write_transfer_waypoint").as_string();
    // this -> csv_to_read_waypoint = this->get_parameter("csv_to_read_waypoint").as_string();
    // this -> csv_to_optimize_waypoint = this->get_parameter("csv_to_optimize_waypoint").as_string();
    csv_to_read_waypoint = "/home/uyen/Drone/drone_ws/src/offboard_control/src/csv_files/waypoint_transfered_list.csv";
    csv_to_optimize_waypoint = "/home/uyen/Drone/drone_ws/src/offboard_control/src/csv_files/minimum_snap_traj.csv";
    // csv_to_read_waypoint = csv_to_write_waypoint;
    // csv_to_read_waypoint = "/home/uyen/Drone/drone_ws/src/offboard_control/src/csv_files/minimum_snap_traj.csv";
    //------------QoS---------------------------
    //----DRONE WAYPOINTS  
    rclcpp::QoS qos_waypoint(rclcpp::KeepLast(100));
    qos_waypoint.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_waypoint.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    //----DRONE POSITION  
    rclcpp::QoS qos_position_global(rclcpp::KeepLast(10));
    qos_position_global.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_position_global.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    //------------Reentrant---------------
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    //---------------Service Server--------------
    service_ = this-> create_service <std_srvs::srv::SetBool>("/offboard_control/process_waypoint_service", 
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
    waypoint_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
        "/offboard_control/waypoints", qos_waypoint,
        std::bind(&ProcessWaypointNode::waypoint_cb, this, _1)
    );

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
    
    if (request->data == true)
    {     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request to Process waypoint server recieved");
    if (optimize_flag)
    {response -> success = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response from Process waypoint server");
    }}
    else
    {RCLCPP_INFO(this->get_logger(), "❗REQUEST RECEIVE, BUT NOT OFFBOARD MODE");}
}



void ProcessWaypointNode::waypoint_cb(const mavros_msgs::msg::WaypointList::SharedPtr msg){
    // RCLCPP_INFO(this->get_logger(), "Received %ld waypoints", msg->waypoints.size());
    if (recieve_wp){return;}
    RCLCPP_INFO(this->get_logger(), "YOU ARE IN WAYPOINT CALLBACK");
    RCLCPP_INFO(this->get_logger(),
        "Got %zu waypoints, current waypoint seq: %u",
        msg->waypoints.size(), msg->current_seq);

    for (int i = 0; i < msg->waypoints.size(); i ++){
        const auto &wp = msg->waypoints[i];
        waypoints_GPS.push_back({wp.x_lat, wp.y_long, wp.z_alt});
    }

    GeographicLib::LocalCartesian proj(org_lat_, org_long_, org_alt_);  // khởi tạo hệ ENU

    for (const auto &gps_point : waypoints_GPS){
        double x,y,z;
        proj.Forward(gps_point[0], gps_point[1], gps_point[2], x, y, z);
        waypoints_ENU.push_back({x,y,z});
    }
   
    recieve_wp = true;
    return;

}

// create a function that used for call python script that calculate minimum snap trajectory for optimizing waypoints
void ProcessWaypointNode::minimum_snap_python(){
    if (optimize_flag) return;

    // ✅ Khởi tạo Python interpreter (chỉ nên gọi 1 lần cho node, nhưng tạm thời giữ nguyên)
    Py_Initialize();
    if (_import_array() < 0) {
    PyErr_Print();
    PyErr_SetString(PyExc_ImportError, "numpy.core.multiarray failed to import");}
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/home/uyen/Drone/drone_ws/src/offboard_control/python_script/Mathematic')");

    // ✅ Import module
    PyObject* pName = PyUnicode_DecodeFSDefault("WaypointOptimization");
    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (!pModule) {
        PyErr_Print();
        std::cerr << "❌ Failed to import Python module WaypointOptimization\n";
        Py_Finalize();
        return;
    }

    // ✅ Lấy class OptimizeWaypoints
    PyObject* pClass = PyObject_GetAttrString(pModule, "OptimizeWaypoints");
    if (!pClass || !PyCallable_Check(pClass)) {
        std::cerr << "❌ Cannot find class OptimizeWaypoints in module\n";
        Py_XDECREF(pClass);
        Py_DECREF(pModule);
        Py_Finalize();
        return;
    }

    // ✅ Tạo instance của class
    PyObject* pInstance = PyObject_CallObject(pClass, nullptr);
    if (!pInstance) {
        PyErr_Print();
        std::cerr << "❌ Failed to instantiate OptimizeWaypoints\n";
        Py_DECREF(pClass);
        Py_DECREF(pModule);
        Py_Finalize();
        return;
    }

    // ✅ Gọi method generate_trajectory_from_csv
    PyObject* pyList = PyList_New(waypoints_ENU.size());
    for (size_t i = 0; i < waypoints_ENU.size(); i ++){
        PyObject* innerList = PyList_New(3);
        PyList_SetItem(innerList, 0, PyFloat_FromDouble(waypoints_ENU[i][0]));
        PyList_SetItem(innerList, 1, PyFloat_FromDouble(waypoints_ENU[i][1]));
        PyList_SetItem(innerList, 2, PyFloat_FromDouble(waypoints_ENU[i][2]));

        // send inner to pylist
        PyList_SetItem(pyList, i, innerList);
    }
    PyObject* pResult = PyObject_CallMethod(
        pInstance,
        "generate_trajectory_from_csv",
        "(Os)",
        pyList,
        csv_to_optimize_waypoint.c_str()
    );

    if (!pResult) {
        PyErr_Print();
        std::cerr << "❌ Error calling generate_trajectory_from_csv()\n";
        Py_DECREF(pInstance);
        Py_DECREF(pClass);
        Py_DECREF(pModule);
        Py_Finalize();
        return;
    }

    std::cout << "✅ Trajectory generated successfully.\n";

    // ✅ Giải nén tuple (t, pos, vel, acc)
    if (PyTuple_Check(pResult) && PyTuple_Size(pResult) >= 4) {
        drone_raw_time = PyTuple_GetItem(pResult, 0);
        drone_raw_pos  = PyTuple_GetItem(pResult, 1);
        drone_raw_vel  = PyTuple_GetItem(pResult, 2);
        drone_raw_acc  = PyTuple_GetItem(pResult, 3);

        // Tăng ref vì PyTuple_GetItem không tự tăng
        Py_INCREF(drone_raw_time);
        Py_INCREF(drone_raw_pos);
        Py_INCREF(drone_raw_vel);
        Py_INCREF(drone_raw_acc);
    } else {
        std::cerr << "⚠️ Python returned invalid tuple.\n";
    }

    Py_DECREF(pResult);

    // ✅ Gọi method check_dynamic_limits(vel, acc)
    PyObject* pCheck = PyObject_CallMethod(pInstance, "check_dynamic_limits", "(OO)", drone_raw_vel, drone_raw_acc);
    if (!pCheck) {
        PyErr_Print();
        std::cerr << "❌ Error calling check_dynamic_limits()\n";
    } else {
        bool in_limit = PyObject_IsTrue(pCheck);
        std::cout << "📊 Dynamic check: " << (in_limit ? "OK" : "Out of limits") << std::endl;
        Py_DECREF(pCheck);
    }

    // ✅ Dọn dẹp
    Py_DECREF(pInstance);
    Py_DECREF(pClass);
    Py_DECREF(pModule);
    Py_Finalize();

    // optimize_flag = true;
    push_waypoint();
    return;
}

void ProcessWaypointNode::push_waypoint(){
    if (!drone_raw_pos){
        RCLCPP_ERROR(this->get_logger(), "❌ No trajectory data to publish.");
        return;
    }

    // Convert Python numpy array to C++ datatype
    PyArrayObject* t_array = (PyArrayObject*)drone_raw_time;
    PyArrayObject* pos_array = (PyArrayObject*)drone_raw_pos;
    PyArrayObject* vel_array = (PyArrayObject*)drone_raw_vel;
    PyArrayObject* acc_array = (PyArrayObject*)drone_raw_acc;

    int num_points = PyArray_DIM(pos_array,0);
    double* time_data = (double*)PyArray_DATA(t_array);
    double* pos_data = (double*)PyArray_DATA(pos_array);
    double* vel_data = (double*)PyArray_DATA(vel_array);
    double* acc_data = (double*)PyArray_DATA(acc_array);

    RCLCPP_INFO(this->get_logger(), "📡 Start publishing %d waypoints to MAVROS...", num_points);

    for (int i = 0; i < num_points; i++){
        double x = pos_data[i*3 + 0];
        double y = pos_data[i*3 + 1];
        double z = pos_data[i*3 + 2];

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
                    //    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE |
                       mavros_msgs::msg::PositionTarget::FORCE);
        target.position.x = y;
        target.position.y = x;
        target.position.z = -z;

        waypoint_processed_pub_->publish(target);
 
    }
    RCLCPP_INFO(this->get_logger(), "✅ Finished publishing all waypoints.");
    optimize_flag = true;
}
void ProcessWaypointNode::main_loop() {
    if (recieved_rq && recieve_wp)
    {
    minimum_snap_python();
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