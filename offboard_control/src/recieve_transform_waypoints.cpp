#include "recieve_transform_waypoints.hpp"

using std::placeholders::_1;
WaypointTransform::WaypointTransform(): Node("waypoints_node")
{

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointTransform>();

    // Instead of spin, use a custom executor
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
