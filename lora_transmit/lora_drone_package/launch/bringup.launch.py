from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include launch của offboard_control
    offboard_pkg_share = get_package_share_directory('offboard_control')
    
    offboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(offboard_pkg_share, 'launch', 'offboard_launch.py')
        )
    )

    # Node LoraDrone (Python)
    lora_drone_node = Node(
        package='lora_drone_package',
        executable='lora_drone',   # trùng console_scripts trong setup.py
        name='lora_drone',
        output='screen',
    )

    return LaunchDescription([
        offboard_launch,
        lora_drone_node
    ])
