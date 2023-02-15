#include "pick_place_app/app/pick_place_task.h"
#include "pick_place_app/skills/utils.h"

#include <tf2_eigen/tf2_eigen.h>

namespace robot_application
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_task");
using namespace std::chrono_literals;

PickPlaceTask::PickPlaceTask(const rclcpp::Node::SharedPtr& node,
                             const PickPlaceTask::Parameters& parameters)
  : node_(node), param_pick_place_(parameters)
{
  parameters_comp_path_skill.loadParameters(node_);
  parameters_io_gripper_skill.loadParameters(node_);
  parameters_detect_aruco_marker_skill.loadParameters(node_);

  moveit_cpp::MoveItCpp::Options moveit_cpp_options(node_);
  moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(node_, moveit_cpp_options);

  initSkills();
}

void PickPlaceTask::initSkills()
{
  comp_path_skill = std::make_shared<robot_skills::ComputePathWithMoveItCppSkill>(
      node_, parameters_comp_path_skill, moveit_cpp_ptr_);

  exec_traj_skill = std::make_shared<robot_skills::ExecuteTrajectorySkill>(node_, moveit_cpp_ptr_);

  io_gripper_skill =
      std::make_shared<robot_skills::IOGripperWithURSkill>(node_, parameters_io_gripper_skill);

  detect_aruco_marker_skill = std::make_shared<robot_skills::DetectArucoMarkerSkill>(
      node_, parameters_detect_aruco_marker_skill);

  RCLCPP_INFO(LOGGER, "Initial skills");
}

void PickPlaceTask::executeTask()
{
  bool step_success = false;
  object_pose_ptr_.reset();
  place_pose_ptr_.reset();

  /**************************
   * Move To Detect Pose *
   **************************/
  comp_path_skill->setPlanner("ompl", "RRTkConfigDefault");
  step_success = comp_path_skill->computePath(
      param_pick_place_.arm_group_name, param_pick_place_.detect_state_name, robot_trajectory);
  if (step_success)
  {
    trajectory_msg = moveit_msgs::msg::RobotTrajectory();
    robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
    RCLCPP_INFO(LOGGER, "-------------- Move to %s, Trajectory size: %ld --------------",
                param_pick_place_.detect_state_name.c_str(),
                trajectory_msg.joint_trajectory.points.size());
    step_success =
        exec_traj_skill->execute_trajectory(param_pick_place_.arm_group_name, robot_trajectory);
    rclcpp::sleep_for(2s);
  }

  /**************************
   * Find Object *
   **************************/
  /** Request Object Pose **/
  if (step_success)
  {
    RCLCPP_INFO(LOGGER, "-------------- Looking for Marker [%d] and [%d] --------------",
                param_pick_place_.object_marker_id, param_pick_place_.place_marker_id);
    std::vector<int> marker_ids{ param_pick_place_.object_marker_id,
                                 param_pick_place_.place_marker_id };
    std::map<int, geometry_msgs::msg::PoseStamped> marker_poses;
    step_success = detect_aruco_marker_skill->getArucoPosestamps(marker_ids, marker_poses);
    if (step_success)
    {
      object_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(
          marker_poses[param_pick_place_.object_marker_id]);
      place_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(
          marker_poses[param_pick_place_.place_marker_id]);
      RCLCPP_INFO(LOGGER, "-------------- Got Marker [%d]: %s \nand [%d]: %s --------------",
                  param_pick_place_.object_marker_id,
                  geometry_msgs::msg::to_yaml(object_pose_ptr_->pose.position).c_str(),
                  param_pick_place_.place_marker_id,
                  geometry_msgs::msg::to_yaml(place_pose_ptr_->pose.position).c_str());
    }
  }

  /**************************
   * Open Hand *
   **************************/

  if (step_success)
  {
    RCLCPP_INFO(LOGGER, "----------------------------\n Open Hand\n----------------------------\n");
    step_success = io_gripper_skill->setGripperState("open");
    rclcpp::sleep_for(1s);
  }

  /**************************
   * Move To Object's Pose *
   **************************/
  /** Move to Prepick Pose above the object **/
  if (step_success)
  {
    geometry_msgs::msg::PointStamped pre_pick_point_msg;
    pre_pick_point_msg.header.frame_id = object_pose_ptr_->header.frame_id;
    pre_pick_point_msg.point = object_pose_ptr_->pose.position;
    pre_pick_point_msg.point.x =
        object_pose_ptr_->pose.position.x + param_pick_place_.pre_pick_offset.at(0);
    pre_pick_point_msg.point.y =
        object_pose_ptr_->pose.position.y + param_pick_place_.pre_pick_offset.at(1);
    pre_pick_point_msg.point.z =
        object_pose_ptr_->pose.position.z + param_pick_place_.pre_pick_offset.at(2);

    trajectory_msg = moveit_msgs::msg::RobotTrajectory();

    RCLCPP_INFO(
        LOGGER,
        "\n----------------------------\n Move to pre PICK pose: %s---------------------------",
        geometry_msgs::msg::to_yaml(pre_pick_point_msg).c_str());

    robot_trajectory->clear();
    moveit_msgs::msg::Constraints path_constraints;
    robot_skills::utils::loadPathConstraintsFromYaml(param_pick_place_.path_constraints_file,
                                                     path_constraints);
    RCLCPP_INFO(LOGGER, "Planning with path_constraints: %s",
                moveit_msgs::msg::to_yaml(path_constraints).c_str());

    comp_path_skill->setPlanner("ompl", "RRTkConfigDefault");
    step_success =
        comp_path_skill->computePath(param_pick_place_.arm_group_name, pre_pick_point_msg,
                                     robot_trajectory, param_pick_place_.ik_frame, false,
                                     path_constraints);

    if (step_success)
    {
      trajectory_msg = moveit_msgs::msg::RobotTrajectory();
      robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
      RCLCPP_INFO(
          LOGGER,
          "\n----------------------------\n Trajectory size:%ld \n ----------------------------",
          trajectory_msg.joint_trajectory.points.size());
      step_success =
          exec_traj_skill->execute_trajectory(param_pick_place_.arm_group_name, robot_trajectory);
    }
    rclcpp::sleep_for(2s);
  }

  if (step_success)
  {
    RCLCPP_INFO(LOGGER,
                "\n----------------------------\n Move to  PICK pose ---------------------------");

    auto direction = geometry_msgs::msg::Vector3();
    direction.z = -param_pick_place_.pre_pick_offset.at(2) + param_pick_place_.pick_offset.at(2);

    std::vector<robot_trajectory::RobotTrajectoryPtr> robot_trajectories;
    step_success = comp_path_skill->computeRelative(param_pick_place_.arm_group_name, direction,
                                                    robot_trajectories, param_pick_place_.ik_frame);
    if (step_success)
    {
      for (const auto& traj : robot_trajectories)
      {
        step_success = exec_traj_skill->execute_trajectory(param_pick_place_.arm_group_name, traj);
      }
    }
    rclcpp::sleep_for(1s);
  }

  /**************************
   * Pick Object *
   **************************/

  /** Allow Collision (hand object) **/

  /** Close hand **/
  if (step_success)
  {
    RCLCPP_INFO(LOGGER,
                "\n----------------------------\n Close Hand \n----------------------------\n");
    step_success = io_gripper_skill->setGripperState("close");
    rclcpp::sleep_for(2s);
  }

  /** Lift object **/
  if (step_success)
  {
    auto direction = geometry_msgs::msg::Vector3();
    direction.z = param_pick_place_.pre_pick_offset.at(2);

    std::vector<robot_trajectory::RobotTrajectoryPtr> robot_trajectories;
    step_success = comp_path_skill->computeRelative(param_pick_place_.arm_group_name, direction,
                                                    robot_trajectories, param_pick_place_.ik_frame);
    if (step_success)
    {
      for (const auto& traj : robot_trajectories)
      {
        step_success = exec_traj_skill->execute_trajectory(param_pick_place_.arm_group_name, traj);
      }
    }
    rclcpp::sleep_for(1s);
  }

  /**************************
   * Move to Place *
   **************************/
  /** Move to pre Place pose **/
  if (step_success)
  {
    comp_path_skill->setPlanner("pilz_industrial_motion_planner", "LIN");

    geometry_msgs::msg::PointStamped pre_place_point_msg;
    pre_place_point_msg.header.frame_id = place_pose_ptr_->header.frame_id;
    pre_place_point_msg.point = place_pose_ptr_->pose.position;
    pre_place_point_msg.point.x =
        place_pose_ptr_->pose.position.x + param_pick_place_.pre_place_offset.at(0);
    pre_place_point_msg.point.y =
        place_pose_ptr_->pose.position.y + param_pick_place_.pre_place_offset.at(1);
    pre_place_point_msg.point.z =
        place_pose_ptr_->pose.position.z + param_pick_place_.pre_place_offset.at(2);
    step_success =
        comp_path_skill->computePath(param_pick_place_.arm_group_name, pre_place_point_msg,
                                     robot_trajectory, param_pick_place_.ik_frame);
    if (step_success)
    {
      robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
      step_success =
          exec_traj_skill->execute_trajectory(param_pick_place_.arm_group_name, robot_trajectory);
    }
    rclcpp::sleep_for(1s);
  }

  if (step_success)
  {
    auto direction = geometry_msgs::msg::Vector3();
    direction.z = -param_pick_place_.pre_place_offset.at(2) + param_pick_place_.place_offset.at(2);

    std::vector<robot_trajectory::RobotTrajectoryPtr> robot_trajectories;
    step_success = comp_path_skill->computeRelative(param_pick_place_.arm_group_name, direction,
                                                    robot_trajectories, param_pick_place_.ik_frame);
    if (step_success)
    {
      for (const auto& traj : robot_trajectories)
      {
        step_success = exec_traj_skill->execute_trajectory(param_pick_place_.arm_group_name, traj);
      }
    }
    rclcpp::sleep_for(1s);
  }

  /** Open hand **/
  if (step_success)
  {
    RCLCPP_INFO(LOGGER,
                "\n----------------------------\n Open Hand \n----------------------------\n");
    step_success = io_gripper_skill->setGripperState("open");
    rclcpp::sleep_for(2s);
  }

  // Set retreat direction

  if (step_success)
  {
    auto direction = geometry_msgs::msg::Vector3();
    direction.z = param_pick_place_.retreat_offset.at(2);
    std::vector<robot_trajectory::RobotTrajectoryPtr> robot_trajectories;
    step_success = comp_path_skill->computeRelative(param_pick_place_.arm_group_name, direction,
                                                    robot_trajectories, param_pick_place_.ik_frame);
    if (step_success)
    {
      for (const auto& traj : robot_trajectories)
      {
        step_success = exec_traj_skill->execute_trajectory(param_pick_place_.arm_group_name, traj);
      }
    }
    rclcpp::sleep_for(1s);
  }

  /**************************
   * Return home *
   **************************/
}

}  // namespace robot_application
