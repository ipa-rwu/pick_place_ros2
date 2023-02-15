# ros2 launch realsense2_camera rs_launch.py device_type:=d435 depth_width:=640 depth_height:=480 depth_fps:=30.0 infra1_width:=640 infra1_height:=480 infra1_fps:=30.0 publish_tf:=false
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    realsence_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "device_type": "d435",
            # "depth_width": 640,
            # "depth_height": 480,
            # "depth_fps": 30.0,
            # "infra1_width": 640,
            # "infra1_height": 480,
            # "infra1_fps": 30.0,
            "publish_tf": "false",
        }.items(),
    )

    realsence_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "-0.00640796",
            "-0.0551158",
            "0.00796752",
            "-0.529877",
            "0.53423",
            "-0.467606",
            "-0.463867",
            "tool0",
            "camera_link",
        ],
    )

    ld = LaunchDescription([realsence_launch, realsence_tf_node])

    return ld
