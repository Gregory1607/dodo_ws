from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the launch file from my_robot_description package
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('my_robot_description'), 'launch', 'display.launch.py')
        ])
    )

    # Launch the differential drive robot node
    diff_drive_node = Node(
        package='my_robot_controller',
        executable='diff_drive_robot',
        name='diff_drive_robot',
        output='screen'
    )

    return LaunchDescription([
        robot_description_launch,
        diff_drive_node
    ])
