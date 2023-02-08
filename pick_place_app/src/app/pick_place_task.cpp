#include "pick_place_app/app/pick_place_task.h"
#include "pick_place_app/skills/utils.h"

#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_eigen/tf2_eigen.h>

namespace robot_application
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_task");
using namespace std::chrono_literals;

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
  errors += !rosparam_shortcuts::get(node, "box_size", box_size);
  errors += !rosparam_shortcuts::get(node, "hand_group_name", hand_group_name);
  errors += !rosparam_shortcuts::get(node, "hand_frame", hand_frame);
  errors += !rosparam_shortcuts::get(node, "path_constraints_file", path_constraints_file);

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

  psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  initSkills();
}

void PickPlaceTask::initSkills()
{
  // comp_path_skill = std::make_shared<robot_skills::ComputePathSkill>(
  //     node_, parameters_comp_path_skill, robot_model_, rm_loader_);
  comp_path_skill = std::make_shared<robot_skills::ComputePathSkill>(
      node_, parameters_comp_path_skill, robot_model_, rm_loader_);
  exec_traj_skill = std::make_shared<robot_skills::ExecuteTrajectorySkill>(node_);
  io_gripper_skill =
      std::make_shared<robot_skills::IOGripperWithURSkill>(node_, parameters_io_gripper_skill);
  modify_planning_scene_skill =
      std::make_shared<robot_skills::ModifyPlanningSceneSkill>(node_, psi_);
  RCLCPP_INFO(LOGGER, "Initial skills");
}

void PickPlaceTask::loadRobot()
{
}

void PickPlaceTask::executeTask()
{
  auto scene_diff_ = planning_scene_->diff();
  bool step_success = false;

  /**************************
   * Move To Detect Pose *
   **************************/
  step_success =
      comp_path_skill->computePath(planning_scene_, pick_place_task_param_.arm_group_name,
                                   pick_place_task_param_.detect_state_name, robot_trajectory);
  if (step_success)
  {
    trajectory_msg = moveit_msgs::msg::RobotTrajectory();
    robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
    RCLCPP_INFO(LOGGER, "-------------- Move to %s, Trajectory size: %ld --------------",
                pick_place_task_param_.detect_state_name.c_str(),
                trajectory_msg.joint_trajectory.points.size());
    step_success =
        exec_traj_skill->execute_trajectory(pick_place_task_param_.arm_group_name, trajectory_msg);
    rclcpp::sleep_for(2s);
  }

  /**************************
   * Find Object *
   **************************/
  /** Request Object Pose **/
  auto object_box = modify_planning_scene_skill->createBox(
      pick_place_task_param_.object_name, pick_place_task_param_.object_frame_id,
      tf2::toMsg(
          Eigen::Isometry3d(Eigen::Translation3d(pick_place_task_param_.object_pose.position.x,
                                                 pick_place_task_param_.object_pose.position.y,
                                                 pick_place_task_param_.object_pose.position.z))),
      pick_place_task_param_.box_size);

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

  // /**************************
  //  * Move To Object's Pose *
  //  **************************/
  // /** Move to Prepick Pose above the object **/
  if (step_success)
  {
    scene_diff_ = planning_scene_->diff();
    geometry_msgs::msg::PointStamped pre_pick_point_msg;
    pre_pick_point_msg.header.frame_id = pick_place_task_param_.object_frame_id;
    pre_pick_point_msg.point.x = pick_place_task_param_.object_pose.position.x;
    pre_pick_point_msg.point.y = pick_place_task_param_.object_pose.position.y;
    pre_pick_point_msg.point.z =
        pick_place_task_param_.object_pose.position.z + pick_place_task_param_.pick_offset;

    trajectory_msg = moveit_msgs::msg::RobotTrajectory();

    RCLCPP_INFO(
        LOGGER,
        "\n----------------------------\n Move to pre PICK pose: %s ----------------------------",
        geometry_msgs::msg::to_yaml(pre_pick_point_msg).c_str());

    robot_trajectory->clear();
    moveit_msgs::msg::Constraints path_constraints;
    robot_skills::utils::loadPathConstraintsFromYaml(pick_place_task_param_.path_constraints_file,
                                                     path_constraints);
    RCLCPP_INFO(LOGGER, "Planning with path_constraints: %s",
                moveit_msgs::msg::to_yaml(path_constraints).c_str());

    step_success =
        comp_path_skill->computePath(planning_scene_, pick_place_task_param_.arm_group_name,
                                     pre_pick_point_msg, robot_trajectory, false, path_constraints);
    if (step_success)
    {
      trajectory_msg = moveit_msgs::msg::RobotTrajectory();
      robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
      RCLCPP_INFO(
          LOGGER,
          "\n----------------------------\n Trajectory size:%ld \n ----------------------------",
          trajectory_msg.joint_trajectory.points.size());
      step_success = exec_traj_skill->execute_trajectory(pick_place_task_param_.arm_group_name,
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
    direction.z = pick_place_task_param_.lift_distance;
    // comp_path_skill->computeRelative(planning_scene_, pick_place_task_param_.arm_group_name,
    //                                  direction, robot_trajectory);
    step_success = comp_path_skill->computeRelative(pick_place_task_param_.arm_group_name,
                                                    direction, robot_trajectory);
    if (step_success)
    {
      robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
      step_success = exec_traj_skill->execute_trajectory(pick_place_task_param_.arm_group_name,
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
    direction.y = pick_place_task_param_.place_pose.position.x -
                  pick_place_task_param_.object_pose.position.x;
    step_success = comp_path_skill->computeRelative(pick_place_task_param_.arm_group_name,
                                                    direction, robot_trajectory);
    if (step_success)
    {
      robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
      step_success = exec_traj_skill->execute_trajectory(pick_place_task_param_.arm_group_name,
                                                         trajectory_msg);
    }
    rclcpp::sleep_for(1s);
  }

  if (step_success)
  {
    auto direction = geometry_msgs::msg::Vector3();
    direction.z = -pick_place_task_param_.pick_offset;
    step_success = comp_path_skill->computeRelative(pick_place_task_param_.arm_group_name,
                                                    direction, robot_trajectory);
    if (step_success)
    {
      robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
      step_success = exec_traj_skill->execute_trajectory(pick_place_task_param_.arm_group_name,
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
