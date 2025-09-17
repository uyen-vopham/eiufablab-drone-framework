#include "offboard_control/drone_transformation.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
#include <functional> 

DroneTransformation::DroneTransformation():Node("drone_transformation_node")
                                                                            
{
    
    //-----------------STATIC TRANSFORM----------------
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    static_transform_.header.stamp = this->get_clock()->now();
    static_transform_.header.frame_id = "base_link";
    static_transform_.child_frame_id = "drone_footprint";

    static_transform_.transform.translation.x = 0.0;
    static_transform_.transform.translation.y = 0.0;
    static_transform_.transform.translation.z = 0.15; 

    static_transform_.transform.rotation.x = 0.0;
    static_transform_.transform.rotation.y = 0.0;
    static_transform_.transform.rotation.z = 0.0;
    static_transform_.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(static_transform_);
    RCLCPP_INFO_STREAM(get_logger(), "Publishing static transform between " 
    <<static_transform_.header.frame_id <<" and " <<static_transform_.child_frame_id);

    //----------------DYNAMIC TRANSFORM
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    //--------------QoS-------------------
    //----DRONE POSE
    rclcpp::QoS qos_pose(rclcpp::KeepLast(10));
    qos_pose.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_pose.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    
    //--------------Subriber--------------
    local_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", qos_pose, 
        std::bind(&DroneTransformation::position_cb, this, _1)
    );

    //--------------Timer-----------------
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DroneTransformation::main_loop, this)
    );

}

void DroneTransformation::position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    dynamic_transform_.header.stamp = this->get_clock()->now();
    dynamic_transform_.header.frame_id = "world";
    dynamic_transform_.child_frame_id = "base_link";
    dynamic_transform_.transform.translation.x = msg->pose.position.x;
    dynamic_transform_.transform.translation.y = msg->pose.position.y;
    dynamic_transform_.transform.translation.z = msg->pose.position.z;
    dynamic_transform_.transform.rotation = msg->pose.orientation;

    dynamic_tf_broadcaster_->sendTransform(dynamic_transform_);

    // RCLCPP_INFO(rclcpp::get_logger("Transform"), "Translation z is %f", msg->pose.position.z);
}

void DroneTransformation::main_loop(){
    // dynamic_transformation();
}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneTransformation>();

    // Instead of spin, use a custom executor
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
