#include "pick_place_app/app/pick_place_static_task.h"
#include "pick_place_app/skills/utils.h"

#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace robot_application
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_static_task");
using namespace std::chrono_literals;

void PickPlaceStaticTask::Parameters::loadParameters(const rclcpp::Node::SharedPtr& node)
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
  errors += !rosparam_shortcuts::get(node, "box_size", box_size);
  errors += !rosparam_shortcuts::get(node, "path_constraints_file", path_constraints_file);
  errors += !rosparam_shortcuts::get(node, "retreat_offset", retreat_offset);

  rosparam_shortcuts::shutdownIfError(errors);
}

PickPlaceStaticTask::PickPlaceStaticTask(const rclcpp::Node::SharedPtr& node,
                                         const PickPlaceStaticTask::Parameters& parameters)
  : node_(node), param_pick_place_static_(parameters)
{
  parameters_comp_path_skill.loadParameters(node_);
  parameters_io_gripper_skill.loadParameters(node_);
  parameters_detect_aruco_marker_skill.loadParameters(node_);

  rm_loader_.reset(new robot_model_loader::RobotModelLoader(node_, "robot_description"));
  // robot_model_ = rm_loader_->getModel();

  initSkills();
}

void PickPlaceStaticTask::initSkills()
{
  comp_path_skill = std::make_shared<robot_skills::ComputePathSkill>(
      node_, parameters_comp_path_skill, rm_loader_);
  RCLCPP_INFO(LOGGER, "Initial skills");
}

void PickPlaceStaticTask::loadRobot()
{
}

void PickPlaceStaticTask::executeTask()
{
  bool step_success = false;

  /**************************
   * Move To Detect Pose *
   **************************/
  // step_success =
  //     comp_path_skill->computePath(param_pick_place_static_.arm_group_name,
  //                                  param_pick_place_static_.detect_state_name, robot_trajectory);
  // if (step_success)
  // {
  //   trajectory_msg = moveit_msgs::msg::RobotTrajectory();
  //   robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
  //   RCLCPP_INFO(LOGGER, "-------------- Move to %s, Trajectory size: %ld --------------",
  //               param_pick_place_static_.detect_state_name.c_str(),
  //               trajectory_msg.joint_trajectory.points.size());
  //   rclcpp::sleep_for(2s);
  // }

  return;
}

}  // namespace robot_application
