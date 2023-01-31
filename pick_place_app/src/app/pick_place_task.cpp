#include "pick_place_app/app/pick_place_task.h"

#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace robot_application
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_task");

void PickPlaceTask::Parameters::loadParameters(const rclcpp::Node::SharedPtr& node)
{
  // Critical parameters (no default exists => shutdown if loading fails)
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(node, "arm_group_name",
                                     arm_group_name);  // e.g. panda_arm
  errors += !rosparam_shortcuts::get(node, "object_pose", object_pose);
  errors += !rosparam_shortcuts::get(node, "place_pose", place_pose);
  errors += !rosparam_shortcuts::get(node, "object_frame_id", object_frame_id);
  errors += !rosparam_shortcuts::get(node, "place_frame_id", place_frame_id);
  errors += !rosparam_shortcuts::get(node, "object_name", object_name);
  errors += !rosparam_shortcuts::get(node, "detect_state_name", detect_state_name);
  errors += !rosparam_shortcuts::get(node, "pick_offset", pick_offset);

  rosparam_shortcuts::shutdownIfError(errors);
}

PickPlaceTask::PickPlaceTask(const rclcpp::Node::SharedPtr& node,
                             const PickPlaceTask::Parameters& parameters)
  : node_(node), pick_place_task_param_(parameters)
{
  parameters_comp_path_skill.loadParameters(node_);
  parameters_io_gripper_skill.loadParameters(node_);

  rm_loader_.reset(new robot_model_loader::RobotModelLoader(node_, "robot_description"));
  robot_model_ = rm_loader_->getModel();

  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

  initSkills();
}

void PickPlaceTask::initSkills()
{
  comp_path_skill = std::make_shared<robot_skills::ComputePathSkill>(
      node_, parameters_comp_path_skill, robot_model_, rm_loader_);
  exec_traj_skill = std::make_shared<robot_skills::ExecuteTrajectorySkill>(node_);
  io_gripper_skill =
      std::make_shared<robot_skills::IOGripperWithURSkill>(node_, parameters_io_gripper_skill);
  RCLCPP_INFO(LOGGER, "Initial skills");
}

void PickPlaceTask::loadRobot()
{
}

void PickPlaceTask::executeTask()
{
  auto scene_diff_ = planning_scene_->diff();

  /**************************
   * Get Ready *
   **************************/

  comp_path_skill->compute(planning_scene_, pick_place_task_param_.arm_group_name,
                           pick_place_task_param_.detect_state_name, robot_trajectory,
                           moveit_msgs::msg::Constraints(), false);

  trajectory_msg = moveit_msgs::msg::RobotTrajectory();
  robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
  RCLCPP_INFO(LOGGER, "-------------- Move to %s, Trajectory size: %ld --------------",
              pick_place_task_param_.detect_state_name.c_str(),
              trajectory_msg.joint_trajectory.points.size());

  exec_traj_skill->execute_trajectory(pick_place_task_param_.arm_group_name, trajectory_msg);

  /**************************
   * Open Hand *
   **************************/
  RCLCPP_INFO(LOGGER, "Connect to %s, Open Hand",
              parameters_io_gripper_skill.io_service_name.c_str());
  io_gripper_skill->setGripperState("open");

  /**************************
   * Move To Object's Pose *
   **************************/
  /** Move to Prepick Pose above the object **/
  scene_diff_ = planning_scene_->diff();
  geometry_msgs::msg::PoseStamped pre_pick_pose_msg;
  pre_pick_pose_msg.header.frame_id = pick_place_task_param_.object_frame_id;
  pre_pick_pose_msg.pose.position = pick_place_task_param_.object_pose.position;

  pre_pick_pose_msg.pose.position.z =
      pick_place_task_param_.object_pose.position.z + pick_place_task_param_.pick_offset;

  trajectory_msg = moveit_msgs::msg::RobotTrajectory();

  comp_path_skill->compute(planning_scene_, pick_place_task_param_.arm_group_name,
                           pre_pick_pose_msg, robot_trajectory, moveit_msgs::msg::Constraints(),
                           true);
  trajectory_msg = moveit_msgs::msg::RobotTrajectory();
  robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
  RCLCPP_INFO(LOGGER,
              "-------------- Move to pre pick pose: %s, Trajectory size:%ld --------------",
              geometry_msgs::msg::to_yaml(pre_pick_pose_msg).c_str(),
              trajectory_msg.joint_trajectory.points.size());

  exec_traj_skill->execute_trajectory(pick_place_task_param_.arm_group_name, trajectory_msg);

  /**************************
   * Pick Object *
   **************************/

  /** Allow Collision (hand object) **/

  /** Close hand **/

  /** Attach object **/

  /** Lift object **/

  /**************************
   * Move to Place *
   **************************/

  /**************************
   * Place Object *
   **************************/

  /** Open Hand **/

  /** Forbid collision(hand,object) **/

  /** Detach Object **/

  // Set retreat direction

  /**************************
   * Return home *
   **************************/
}

}  // namespace robot_application
