from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mock_io_server = Node(
        package="pick_place_app",
        executable="mock_io_server",
        output="screen",
    )

    return LaunchDescription([mock_io_server])
