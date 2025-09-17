#include "offboard_control/marker_publish.hpp"

using std::placeholders::_1;
MarkerPublish::MarkerPublish(): Node("marker_node"),published_flag_(false),
                                                    csv_path_("/home/uyen/Drone/drone_ws/src/offboard_control/src/waypoint_log.csv")
{
    //----------QOS setting----------
    //PUBLISH
    rclcpp::QoS qos_pub(rclcpp::KeepLast(10));
    qos_pub.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_pub.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


    //--------PUBLISH----------------
    point_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/marker_array", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/uyen_path", 10);


    //----------TIMER----------------
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MarkerPublish::main_loop_, this)
    );

    //---------CSV PATH--------------
    // std::vector<std::array<float, 3>> points = read_csv("/home/uyen/Drone/drone_ws/src/offboard_control/src/waypoint_log.csv");
    
}

void MarkerPublish::publish_points_(const std::vector<geometry_msgs::msg::Point>& points){
   if (published_flag_)
    {return;}


    ///Draw a single point/////
    // marker_.header.frame_id = "odom";
    // marker_.header.stamp = this->get_clock()->now();
    // marker_.type = visualization_msgs::msg::Marker::SPHERE;
    // marker_.action = visualization_msgs::msg::Marker::ADD;
    // marker_.id = 0;

    // marker_.pose.position = points[0];  // Lấy điểm đầu tiên
    // marker_.scale.x = 2.5;
    // marker_.scale.y = 2.5;
    // marker_.scale.z = 2.5;
    // marker_.color.a = 1.0;
    // marker_.color.r = 0.0;
    // marker_.color.g = 1.0;
    // marker_.color.b = 0.0;
    // RCLCPP_INFO(this->get_logger(), "YOU ARE IN WAYPOINT CALLBACK");

    // marker_array_.markers.clear();              // Xóa các marker cũ
    // marker_array_.markers.push_back(marker_);   // Thêm 1 điểm duy nhất

    // point_pub_->publish(marker_array_);
    // published_flag_ = true;

    //Draw points//
    int id = 0;

    for (const auto& p : points)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        // marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.type = 2;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.id = id++;  // Mỗi marker phải có ID riêng
        // print("x: ", p.x)
        RCLCPP_INFO(this->get_logger(), "YOU ARE IN WAYPOINT CALLBACK %f", p.x);
        RCLCPP_INFO(this->get_logger(), "YOU ARE IN WAYPOINT CALLBACK %f", p.y);
        RCLCPP_INFO(this->get_logger(), "YOU ARE IN WAYPOINT CALLBACK %f", p.z);
        
        marker.pose.position = p;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker_array_.markers.push_back(marker);
    }
    
    point_pub_->publish(marker_array_);
    published_flag_ = true;

}

void MarkerPublish::publish_path_(const std::vector<geometry_msgs::msg::Point>& points){
    // if (published_flag_){return;}

    // std::cout << "Danh sách điểm nhận được (" << points.size() << " điểm):" << std::endl;
    // for (size_t i = 0; i < points.size(); ++i) 
    // {
    //     const auto& p = points[i];
    //     std::cout << "  Point " << i + 1 << ": (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
    // }

    
    nav_msgs::msg::Path path_msg_;
    path_msg_.header.frame_id = "map";
    path_msg_.header.stamp = this -> get_clock()->now();

    for (const auto& p : points)
    {
        // std::cout << "điểm nhận được (" << p.x <" "<< p.y << "" << p.z << " điểm):" << std::endl;
        // printf("Điểm nhận được: (x = %.2f, y = %.2f, z = %.2f)\n", p.x, p.y, p.z);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this -> get_clock() -> now();
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.position.z = p.z;
        pose.pose.orientation.w = 1.0;
        path_msg_.poses.push_back(pose);
        path_msg_.poses.push_back(pose);

        std::cout << "Pose vừa thêm: (x = " << pose.pose.position.x
                << ", y = " << pose.pose.position.y
                << ", z = " << pose.pose.position.z
                << "), orientation (w = " << pose.pose.orientation.w << ")"
                << std::endl;


            }

    path_pub_->publish(path_msg_);
    published_flag_ = true;
}

std::vector<geometry_msgs::msg::Point> MarkerPublish::read_csv_(const std::string& csv_path){
    std::vector<geometry_msgs::msg::Point> points;
    std::ifstream file(csv_path);
    std::string line;
    while (std::getline(file, line)){
        std::stringstream ss(line);
        std::string id_str, x_str, y_str, z_str;

        std::getline(ss, id_str, ',');
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, z_str, ',');

        if (!x_str.empty() && !y_str.empty() && !z_str.empty()){
            geometry_msgs::msg::Point p;
            p.x = std::stod(x_str);
            p.y = std::stod(y_str);
            p.z = std::stod(z_str);
            points.push_back(p);
            // RCLCPP_INFO(this->get_logger(), "the value of x is %f", p.x);
        }
    }
    return points;
}

void MarkerPublish::main_loop_(){
    // if (!published_flag_){
    auto points = read_csv_(csv_path_);
    std::cout << "points:" << std::endl;

    publish_points_(points);
    publish_path_(points);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerPublish>();

    // Instead of spin, use a custom executor
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
