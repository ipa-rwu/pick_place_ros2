import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .moveit_cpp(
            file_path=get_package_share_directory("pick_place_app")
            + "/config/moveitcpp.yaml"
        )
        .to_moveit_configs()
    )

    point_to_point_demo = Node(
        package="pick_place_app",
        executable="point_to_point_demo",
        name="point_to_point_task",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("pick_place_app"),
                "config",
                "point_to_point_panda.yaml",
            ),
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([point_to_point_demo])
