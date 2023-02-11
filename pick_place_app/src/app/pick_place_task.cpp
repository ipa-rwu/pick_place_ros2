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
  errors += !rosparam_shortcuts::get(node, "detect_state_name", detect_state_name);
  errors += !rosparam_shortcuts::get(node, "pick_offset", pick_offset);
  errors += !rosparam_shortcuts::get(node, "place_offset", place_offset);
  errors += !rosparam_shortcuts::get(node, "box_size", box_size);
  errors += !rosparam_shortcuts::get(node, "object_marker_id", object_marker_id);
  errors += !rosparam_shortcuts::get(node, "place_marker_id", place_marker_id);
  errors += !rosparam_shortcuts::get(node, "path_constraints_file", path_constraints_file);
  errors += !rosparam_shortcuts::get(node, "retreat_offset", retreat_offset);

  rosparam_shortcuts::shutdownIfError(errors);
}

PickPlaceTask::PickPlaceTask(const rclcpp::Node::SharedPtr& node,
                             const PickPlaceTask::Parameters& parameters)
  : node_(node), pick_place_task_param_(parameters)
{
  parameters_comp_path_skill.loadParameters(node_);
  parameters_io_gripper_skill.loadParameters(node_);
  parameters_detect_aruco_marker_skill.loadParameters(node_);

  rm_loader_.reset(new robot_model_loader::RobotModelLoader(node_, "robot_description"));

  psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  psm_.reset(new planning_scene_monitor::PlanningSceneMonitor(node, "robot_description"));

  psm_->startSceneMonitor();
  psm_->startWorldGeometryMonitor();
  psm_->startStateMonitor();

  initSkills();
}

void PickPlaceTask::initSkills()
{
  comp_path_skill = std::make_shared<robot_skills::ComputePathSkill>(
      node_, parameters_comp_path_skill, rm_loader_, psm_);
  exec_traj_skill = std::make_shared<robot_skills::ExecuteTrajectorySkill>(node_);
  io_gripper_skill =
      std::make_shared<robot_skills::IOGripperWithURSkill>(node_, parameters_io_gripper_skill);
  modify_planning_scene_skill =
      std::make_shared<robot_skills::ModifyPlanningSceneSkill>(node_, psi_);
  detect_aruco_marker_skill = std::make_shared<robot_skills::DetectArucoMarkerSkill>(
      node_, parameters_detect_aruco_marker_skill);

  RCLCPP_INFO(LOGGER, "Initial skills");
}

void PickPlaceTask::loadRobot()
{
}

void PickPlaceTask::executeTask()
{
  auto scene_diff_ = planning_scene_->diff();
  bool step_success = false;
  object_pose_ptr_.reset();
  place_pose_ptr_.reset();

  /**************************
   * Move To Detect Pose *
   **************************/
  step_success =
      comp_path_skill->computePath(pick_place_task_param_.arm_group_name,
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
  if (step_success)
  {
    std::vector<int> marker_ids{ pick_place_task_param_.object_marker_id,
                                 pick_place_task_param_.place_marker_id };
    std::map<int, geometry_msgs::msg::PoseStamped> marker_poses;
    step_success = detect_aruco_marker_skill->getArucoPosestamps(marker_ids, marker_poses);
    if (step_success)
    {
      object_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(
          marker_poses[pick_place_task_param_.object_marker_id]);
      place_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(
          marker_poses[pick_place_task_param_.place_marker_id]);
      // add object
      auto object_box =
          modify_planning_scene_skill->createBox("box_1", object_pose_ptr_->header.frame_id,
                                                 object_pose_ptr_->pose,
                                                 pick_place_task_param_.box_size);
      modify_planning_scene_skill->addObject(object_box);

      std::vector<double> place_box_size{ 0.020, 0.020, 0.001 };
      auto place_box = modify_planning_scene_skill->createBox(
          "box_2", place_pose_ptr_->header.frame_id, object_pose_ptr_->pose, place_box_size);
      modify_planning_scene_skill->addObject(place_box);
    }
  }

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
    // pre_pick_point_msg.header.frame_id = object_pose_ptr_->header.frame_id;
    // pre_pick_point_msg.point.x = object_pose_ptr_->pose.position.x;
    // pre_pick_point_msg.point.y = object_pose_ptr_->pose.position.y;
    // pre_pick_point_msg.point.z =
    //     object_pose_ptr_->pose.position.z + pick_place_task_param_.pick_offset;

    pre_pick_point_msg.header.frame_id = "world";
    pre_pick_point_msg.point.x = -0.188595;
    pre_pick_point_msg.point.y = 0.293224;
    pre_pick_point_msg.point.z = 1.12;

    trajectory_msg = moveit_msgs::msg::RobotTrajectory();

    RCLCPP_INFO(
        LOGGER,
        "\n----------------------------\n Move to pre PICK pose: %s---------------------------",
        geometry_msgs::msg::to_yaml(pre_pick_point_msg).c_str());

    robot_trajectory->clear();
    moveit_msgs::msg::Constraints path_constraints;
    // robot_skills::utils::loadPathConstraintsFromYaml(pick_place_task_param_.path_constraints_file,
    //                                                  path_constraints);
    // RCLCPP_INFO(LOGGER, "Planning with path_constraints: %s",
    //             moveit_msgs::msg::to_yaml(path_constraints).c_str());

    step_success =
        comp_path_skill->computePath(pick_place_task_param_.arm_group_name, pre_pick_point_msg,
                                     robot_trajectory, parameters_comp_path_skill.ik_frame, false,
                                     path_constraints);
    int i = 0;

    while (step_success == false && i < 10)
    {
      step_success =
          comp_path_skill->computePath(pick_place_task_param_.arm_group_name, pre_pick_point_msg,
                                       robot_trajectory, parameters_comp_path_skill.ik_frame, false,
                                       path_constraints);
      i++;
      rclcpp::sleep_for(1s);
      RCLCPP_ERROR(LOGGER, "Try %d times", i);
    }

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
    direction.x = object_pose_ptr_->pose.position.x - place_pose_ptr_->pose.position.x;
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
    direction.y = object_pose_ptr_->pose.position.y - place_pose_ptr_->pose.position.y;

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
    direction.z = object_pose_ptr_->pose.position.z - place_pose_ptr_->pose.position.z +
                  pick_place_task_param_.place_offset;
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

  // Set retreat direction

  if (step_success)
  {
    auto direction = geometry_msgs::msg::Vector3();
    direction.z = pick_place_task_param_.retreat_offset;
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
   * Return home *
   **************************/
}

}  // namespace robot_application
