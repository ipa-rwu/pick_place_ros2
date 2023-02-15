import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur5e_workcell", package_name="ur5e_cell_moveit_config")
        .robot_description(file_path="config/ur5e_workcell.urdf.xacro")
        .moveit_cpp(
            file_path=get_package_share_directory("pick_place_app")
            + "/config/moveitcpp.yaml"
        )
        .to_moveit_configs()
    )

    pick_place_demo = Node(
        package="pick_place_app",
        executable="pick_place_task_demo",
        output="screen",
        # prefix=["gdb -ex run --args"],
        # prefix=["xterm -e gdb -ex run --args"],
        parameters=[
            os.path.join(
                get_package_share_directory("pick_place_app"),
                "config",
                "pick_place_task_ur5e.yaml",
            ),
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([pick_place_demo])
