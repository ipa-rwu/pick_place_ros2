#include "pick_place_app/app/point_to_point.h"
#include "pick_place_app/skills/utils.h"

#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <geometry_msgs/msg/pose_array.h>
#include <moveit_msgs/msg/collision_object.h>

#include <tf2_eigen/tf2_eigen.h>

namespace robot_application
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("point_to_point");
using namespace std::chrono_literals;

void PointToPoinTask::Parameters::loadParameters(const rclcpp::Node::SharedPtr& node)
{
  // Critical parameters (no default exists => shutdown if loading fails)
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(node, "arm_group_name",
                                     arm_group_name);  // e.g. arm
  errors += !rosparam_shortcuts::get(node, "arm_hand_group_name",
                                     arm_hand_group_name);  // e.g. arm_hand
  errors += !rosparam_shortcuts::get(node, "waypoints_file", waypoints_file);
  errors += !rosparam_shortcuts::get(node, "start_state", start_state);
  errors += !rosparam_shortcuts::get(node, "offset", offset);
  errors += !rosparam_shortcuts::get(node, "box_size", box_size);
  errors += !rosparam_shortcuts::get(node, "ik_frame", ik_frame);

  rosparam_shortcuts::shutdownIfError(errors);
}

PointToPoinTask::PointToPoinTask(const rclcpp::Node::SharedPtr& node,
                                 const PointToPoinTask::Parameters& parameters)
  : node_(node), param_task_(parameters)
{
  parameters_comp_path_skill.loadParameters(node_);

  rm_loader_.reset(new robot_model_loader::RobotModelLoader(node_, "robot_description"));
  robot_model_ = rm_loader_->getModel();

  psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  initSkills();
}

void PointToPoinTask::initSkills()
{
  comp_path_skill = std::make_shared<robot_skills::ComputePathSkill>(
      node_, parameters_comp_path_skill, robot_model_, rm_loader_);
  exec_traj_skill = std::make_shared<robot_skills::ExecuteTrajectorySkill>(node_);
  modify_planning_scene_skill =
      std::make_shared<robot_skills::ModifyPlanningSceneSkill>(node_, psi_);

  RCLCPP_INFO(LOGGER, "Initial skills");
}

void PointToPoinTask::loadRobot()
{
}

void PointToPoinTask::executeTask()
{
  bool step_success = false;

  /**************************
   * Move To Initial Pose *
   **************************/
  step_success = comp_path_skill->computePath(param_task_.arm_group_name, param_task_.start_state,
                                              robot_trajectory_);
  if (step_success)
  {
    trajectory_msg_ = moveit_msgs::msg::RobotTrajectory();
    robot_trajectory_->getRobotTrajectoryMsg(trajectory_msg_);
    RCLCPP_INFO(LOGGER, "-------------- Move to %s, Trajectory size: %ld --------------",
                param_task_.start_state.c_str(), trajectory_msg_.joint_trajectory.points.size());
    step_success = exec_traj_skill->execute_trajectory(param_task_.arm_group_name, trajectory_msg_);
    rclcpp::sleep_for(2s);
  }

  /**************************
   * Load Waypoints from File *
   **************************/
  geometry_msgs::msg::PoseArray pose_array;
  moveit_msgs::msg::CollisionObject point_box;
  geometry_msgs::msg::PointStamped point_msg;

  robot_skills::utils::loadPointsFromYaml(param_task_.waypoints_file, pose_array);
  auto frame_id = pose_array.header.frame_id;
  for (int i = 0; i < pose_array.poses.size(); i++)
  {
    // add object
    point_box = modify_planning_scene_skill->createBox(
        "point_" + std::to_string(i), frame_id, pose_array.poses.at(i), param_task_.box_size);

    modify_planning_scene_skill->addObject(point_box);

    point_msg.header.frame_id = frame_id;
    point_msg.point = pose_array.poses.at(i).position;
    point_msg.point.z = point_msg.point.z + param_task_.offset;

    trajectory_msg_ = moveit_msgs::msg::RobotTrajectory();

    step_success = comp_path_skill->computePath(param_task_.arm_group_name, point_msg,
                                                robot_trajectory_, param_task_.ik_frame);

    if (step_success)
    {
      trajectory_msg_ = moveit_msgs::msg::RobotTrajectory();
      robot_trajectory_->getRobotTrajectoryMsg(trajectory_msg_);
      RCLCPP_INFO(
          LOGGER,
          "\n----------------------------\n Trajectory size:%ld \n ----------------------------",
          trajectory_msg_.joint_trajectory.points.size());
      step_success =
          exec_traj_skill->execute_trajectory(param_task_.arm_group_name, trajectory_msg_);
    }
    else if (i > 0)
    {
      auto direction = geometry_msgs::msg::Vector3();
      direction.x = pose_array.poses.at(i).position.x - pose_array.poses.at(i - 1).position.x;
      direction.y = pose_array.poses.at(i).position.y - pose_array.poses.at(i - 1).position.y;
      direction.z = pose_array.poses.at(i).position.z - pose_array.poses.at(i - 1).position.z;
      step_success = comp_path_skill->computeRelative(param_task_.arm_group_name, direction,
                                                      robot_trajectory_);
      if (step_success)
      {
        trajectory_msg_ = moveit_msgs::msg::RobotTrajectory();
        robot_trajectory_->getRobotTrajectoryMsg(trajectory_msg_);
        RCLCPP_INFO(
            LOGGER,
            "\n----------------------------\n Trajectory size:%ld \n ----------------------------",
            trajectory_msg_.joint_trajectory.points.size());
        step_success =
            exec_traj_skill->execute_trajectory(param_task_.arm_group_name, trajectory_msg_);
      }
    }

    rclcpp::sleep_for(1s);
  }
}

}  // namespace robot_application
