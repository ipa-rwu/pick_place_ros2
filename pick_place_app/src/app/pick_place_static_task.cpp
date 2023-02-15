#include "pick_place_app/app/pick_place_static_task.h"
#include "pick_place_app/skills/utils.h"

namespace robot_application
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_static_task");
using namespace std::chrono_literals;

PickPlaceStaticTask::PickPlaceStaticTask(const rclcpp::Node::SharedPtr& node,
                                         const PickPlaceStaticTask::Parameters& parameters)
  : node_(node), param_pick_place_static_(parameters)
{
  parameters_comp_path_skill.loadParameters(node_);
  parameters_io_gripper_skill.loadParameters(node_);

  moveit_cpp::MoveItCpp::Options moveit_cpp_options(node_);
  moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(node_, moveit_cpp_options);

  initSkills();
}

void PickPlaceStaticTask::initSkills()
{
  comp_path_skill = std::make_shared<robot_skills::ComputePathWithMoveItCppSkill>(
      node_, parameters_comp_path_skill, moveit_cpp_ptr_);
  exec_traj_skill = std::make_shared<robot_skills::ExecuteTrajectorySkill>(node_, moveit_cpp_ptr_);
  io_gripper_skill =
      std::make_shared<robot_skills::IOGripperWithURSkill>(node_, parameters_io_gripper_skill);

  psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  // require ApplyPlanningScene service from move_group
  modify_planning_scene_skill =
      std::make_shared<robot_skills::ModifyPlanningSceneSkill>(node_, psi_);

  RCLCPP_INFO(LOGGER, "Initial skills");
}

void PickPlaceStaticTask::executeTask()
{
  bool step_success = false;

  /**************************
   * Move To Detect Pose *
   **************************/
  comp_path_skill->setPlanner("ompl", "RRTkConfigDefault");
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
                                                       robot_trajectory);
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

    comp_path_skill->setPlanner("ompl", "RRTkConfigDefault");
    step_success =
        comp_path_skill->computePath(param_pick_place_static_.arm_group_name, pre_pick_point_msg,
                                     robot_trajectory, param_pick_place_static_.ik_frame, false,
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
                                                         robot_trajectory);
    }
    rclcpp::sleep_for(1s);
  }

  /**************************
   * Pick Object *
   **************************/

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

    std::vector<robot_trajectory::RobotTrajectoryPtr> robot_trajectories;
    step_success =
        comp_path_skill->computeRelative(param_pick_place_static_.arm_group_name, direction,
                                         robot_trajectories, param_pick_place_static_.ik_frame);
    if (step_success)
    {
      for (const auto& traj : robot_trajectories)
      {
        step_success =
            exec_traj_skill->execute_trajectory(param_pick_place_static_.arm_group_name, traj);
      }
    }

    rclcpp::sleep_for(1s);
  }

  /**************************
   * Move to Place *
   **************************/
  if (step_success)
  {
    comp_path_skill->setPlanner("pilz_industrial_motion_planner", "LIN");
    geometry_msgs::msg::PointStamped pre_place_point_msg;
    pre_place_point_msg.header.frame_id = param_pick_place_static_.place_frame_id;
    pre_place_point_msg.point = param_pick_place_static_.place_pose.position;
    pre_place_point_msg.point.z =
        pre_place_point_msg.point.z + param_pick_place_static_.place_offset;
    step_success =
        comp_path_skill->computePath(param_pick_place_static_.arm_group_name, pre_place_point_msg,
                                     robot_trajectory, param_pick_place_static_.ik_frame);
    if (step_success)
    {
      robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);
      step_success = exec_traj_skill->execute_trajectory(param_pick_place_static_.arm_group_name,
                                                         robot_trajectory);
    }
    rclcpp::sleep_for(1s);
  }

  if (step_success)
  {
    auto direction = geometry_msgs::msg::Vector3();
    direction.z = -param_pick_place_static_.pick_offset;

    std::vector<robot_trajectory::RobotTrajectoryPtr> robot_trajectories;
    step_success =
        comp_path_skill->computeRelative(param_pick_place_static_.arm_group_name, direction,
                                         robot_trajectories, param_pick_place_static_.ik_frame);
    if (step_success)
    {
      for (const auto& traj : robot_trajectories)
      {
        step_success =
            exec_traj_skill->execute_trajectory(param_pick_place_static_.arm_group_name, traj);
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
    rclcpp::sleep_for(1s);
  }

  // Set retreat direction
  if (step_success)
  {
    auto direction = geometry_msgs::msg::Vector3();
    direction.z = param_pick_place_static_.retreat_offset;

    std::vector<robot_trajectory::RobotTrajectoryPtr> robot_trajectories;
    step_success =
        comp_path_skill->computeRelative(param_pick_place_static_.arm_group_name, direction,
                                         robot_trajectories, param_pick_place_static_.ik_frame);
    if (step_success)
    {
      for (const auto& traj : robot_trajectories)
      {
        step_success =
            exec_traj_skill->execute_trajectory(param_pick_place_static_.arm_group_name, traj);
      }
    }

    rclcpp::sleep_for(1s);
  }
  /**************************
   * Return home *
   **************************/
}

}  // namespace robot_application
