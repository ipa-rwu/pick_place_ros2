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
  robot_model_ = rm_loader_->getModel();

  psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  psm_.reset(new planning_scene_monitor::PlanningSceneMonitor(node, "robot_description"));

  psm_->startSceneMonitor();
  psm_->startWorldGeometryMonitor();
  psm_->startStateMonitor();

  initSkills();
}

void PickPlaceStaticTask::initSkills()
{
  comp_path_skill = std::make_shared<robot_skills::ComputePathSkill>(
      node_, parameters_comp_path_skill, rm_loader_, psm_);
  exec_traj_skill = std::make_shared<robot_skills::ExecuteTrajectorySkill>(node_);
  io_gripper_skill =
      std::make_shared<robot_skills::IOGripperWithURSkill>(node_, parameters_io_gripper_skill);
  modify_planning_scene_skill =
      std::make_shared<robot_skills::ModifyPlanningSceneSkill>(node_, psi_);

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
  step_success =
      comp_path_skill->computePath(param_pick_place_static_.arm_group_name,
                                   param_pick_place_static_.detect_state_name, robot_trajectory);
  if (step_success)
  {
    trajectory_msg = moveit_msgs::msg::RobotTrajectory();
    robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
    RCLCPP_INFO(LOGGER, "-------------- Move to %s, Trajectory size: %ld --------------",
                param_pick_place_static_.detect_state_name.c_str(),
                trajectory_msg.joint_trajectory.points.size());
    step_success = exec_traj_skill->execute_trajectory(param_pick_place_static_.arm_group_name,
                                                       trajectory_msg);
    rclcpp::sleep_for(2s);
  }

  /**************************
   * Get Object Pose from Param *
   **************************/
  /** Request Object Pose **/

  auto object_box = modify_planning_scene_skill->createBox(
      param_pick_place_static_.object_name, param_pick_place_static_.object_frame_id,
      tf2::toMsg(
          Eigen::Isometry3d(Eigen::Translation3d(param_pick_place_static_.object_pose.position.x,
                                                 param_pick_place_static_.object_pose.position.y,
                                                 param_pick_place_static_.object_pose.position.z))),
      param_pick_place_static_.box_size);

  modify_planning_scene_skill->addObject(object_box);

  /**************************
   * Open Hand *
   **************************/
  RCLCPP_INFO(LOGGER, "----------------------------\n Open Hand\n----------------------------\n");

  if (step_success)
  {
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
    pre_pick_point_msg.header.frame_id = param_pick_place_static_.object_frame_id;
    pre_pick_point_msg.point.x = param_pick_place_static_.object_pose.position.x;
    pre_pick_point_msg.point.y = param_pick_place_static_.object_pose.position.y;
    pre_pick_point_msg.point.z =
        param_pick_place_static_.object_pose.position.z + param_pick_place_static_.pick_offset;

    trajectory_msg = moveit_msgs::msg::RobotTrajectory();

    RCLCPP_INFO(
        LOGGER,
        "\n----------------------------\n Move to pre PICK pose: %s---------------------------",
        geometry_msgs::msg::to_yaml(pre_pick_point_msg).c_str());

    robot_trajectory->clear();
    moveit_msgs::msg::Constraints path_constraints;
    if (robot_skills::utils::loadPathConstraintsFromYaml(
            param_pick_place_static_.path_constraints_file, path_constraints))
    {
      RCLCPP_INFO(LOGGER, "Planning with path_constraints: %s",
                  moveit_msgs::msg::to_yaml(path_constraints).c_str());
    }

    step_success =
        comp_path_skill->computePath(param_pick_place_static_.arm_group_name, pre_pick_point_msg,
                                     robot_trajectory, parameters_comp_path_skill.ik_frame, false,
                                     path_constraints);
    if (step_success)
    {
      trajectory_msg = moveit_msgs::msg::RobotTrajectory();
      robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
      RCLCPP_INFO(
          LOGGER,
          "\n----------------------------\n Trajectory size:%ld \n ----------------------------",
          trajectory_msg.joint_trajectory.points.size());
      step_success = exec_traj_skill->execute_trajectory(param_pick_place_static_.arm_group_name,
                                                         trajectory_msg);
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
    rclcpp::sleep_for(1s);
  }

  /** Attach object **/
  // modify_planning_scene_skill->attach_objects_(object_box);

  /** Lift object **/
  if (step_success)
  {
    auto direction = geometry_msgs::msg::Vector3();
    direction.z = param_pick_place_static_.lift_distance;
    step_success = comp_path_skill->computeRelative(param_pick_place_static_.arm_group_name,
                                                    direction, robot_trajectory);
    if (step_success)
    {
      robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
      step_success = exec_traj_skill->execute_trajectory(param_pick_place_static_.arm_group_name,
                                                         trajectory_msg);
    }
    rclcpp::sleep_for(1s);
  }

  /**************************
   * Move to Place *
   **************************/
  if (step_success)
  {
    auto direction = geometry_msgs::msg::Vector3();
    direction.y = param_pick_place_static_.place_pose.position.x -
                  param_pick_place_static_.object_pose.position.x;
    step_success = comp_path_skill->computeRelative(param_pick_place_static_.arm_group_name,
                                                    direction, robot_trajectory);
    if (step_success)
    {
      robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
      step_success = exec_traj_skill->execute_trajectory(param_pick_place_static_.arm_group_name,
                                                         trajectory_msg);
    }
    rclcpp::sleep_for(1s);
  }

  if (step_success)
  {
    auto direction = geometry_msgs::msg::Vector3();
    direction.z = -param_pick_place_static_.pick_offset;
    step_success = comp_path_skill->computeRelative(param_pick_place_static_.arm_group_name,
                                                    direction, robot_trajectory);
    if (step_success)
    {
      robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
      step_success = exec_traj_skill->execute_trajectory(param_pick_place_static_.arm_group_name,
                                                         trajectory_msg);
    }
    rclcpp::sleep_for(1s);
  }

  /** Open hand **/
  if (step_success)
  {
    RCLCPP_INFO(LOGGER,
                "\n----------------------------\n Open Hand \n----------------------------\n");
    step_success = io_gripper_skill->setGripperState("open");
    rclcpp::sleep_for(1s);
  }

  // Set retreat direction
  if (step_success)
  {
    auto direction = geometry_msgs::msg::Vector3();
    direction.z = param_pick_place_static_.retreat_offset;
    step_success = comp_path_skill->computeRelative(param_pick_place_static_.arm_group_name,
                                                    direction, robot_trajectory);
    if (step_success)
    {
      robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
      step_success = exec_traj_skill->execute_trajectory(param_pick_place_static_.arm_group_name,
                                                         trajectory_msg);
    }
    rclcpp::sleep_for(1s);
  }
  /**************************
   * Return home *
   **************************/
}

}  // namespace robot_application
