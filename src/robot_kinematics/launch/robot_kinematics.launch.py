from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    rosbag_path = '/home/jean/ros2_bag/rosbag2_2025_01_08-14_00_08/'

    return LaunchDescription([
        Node(
            package="robot_kinematics",
            executable="node-robotKinematics"
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', rosbag_path],
        ),
    ])
