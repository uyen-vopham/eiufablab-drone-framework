from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    process_waypoint = Node (package = "offboard_control",
                              executable="process_waypoint_node",
                              name = "process_waypoint_node",
                              output="screen",
                              parameters=[{"csv_to_write_waypoint":"/home/uyen/Drone/drone_ws/src/offboard_control/src/csv_files/waypoint_list.csv"},
                                          {"csv_to_read_waypoint": "/home/uyen/Drone/drone_ws/src/offboard_control/src/csv_files/minimum_snap_traj.csv"},
                                          {"csv_to_write_transfer_waypoint": "/home/uyen/Drone/drone_ws/src/offboard_control/src/csv_files/waypoint_to_optimize.csv"}]

    )
    offboard_control = Node (package = "offboard_control",
                            executable="offboard_server_node",
                            name = "offboard_server_node",
                            output="screen",
        
    )
    drone_transformation = Node (package = "offboard_control",
                                 executable="drone_transformation_node",
                                 name = "drone_transformation_node",
                                 output = "screen"
        
    )
    # lora_transmit_node = Node (package="offboard_control",
    #                     executable="lora_transmit.py",
    #                     name = "LoraDrone",
    #                     output = "screen"
    # )
    return LaunchDescription([process_waypoint])#, offboard_control, drone_transformation])