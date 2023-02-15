from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")

    use_fake_hardware_launch_arg = DeclareLaunchArgument(
        "use_fake_hardware", default_value="true"
    )

    robot_ip_launch_arg = DeclareLaunchArgument(
        "robot_ip", default_value="192.168.56.2"
    )

    launch_rviz_arg = DeclareLaunchArgument("launch_rviz", default_value="true")

    ur_control_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "use_fake_hardware": use_fake_hardware,
            "initial_joint_controller": "joint_trajectory_controller",
            "activate_joint_controller": "true",
            "robot_ip": robot_ip,
            "description_package": "ur5e_cell_description",
            "description_file": "workcell.urdf.xacro",
            "launch_rviz": launch_rviz,
        }.items(),
    )

    ld = LaunchDescription(
        [
            use_fake_hardware_launch_arg,
            robot_ip_launch_arg,
            launch_rviz_arg,
            ur_control_launch,
        ]
    )

    return ld
