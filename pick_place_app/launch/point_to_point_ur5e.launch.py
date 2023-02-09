import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "ur5e_workcell_fake", package_name="ur5e_cell_moveit_config"
    ).to_dict()

    point_to_point_demo = Node(
        package="pick_place_app",
        executable="point_to_point_demo",
        output="screen",
        # prefix=["gdb -ex run --args"],
        # prefix=["xterm -e gdb -ex run --args"],
        parameters=[
            os.path.join(
                get_package_share_directory("pick_place_app"),
                "config",
                "point_to_point_ur5e.yaml",
            ),
            moveit_config,
        ],
    )

    return LaunchDescription([point_to_point_demo])
