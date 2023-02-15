#include "pick_place_app/app/point_to_point.h"
#include "pick_place_app/skills/utils.h"

#include <geometry_msgs/msg/pose_array.h>
#include <moveit_msgs/msg/collision_object.h>

#include <tf2_eigen/tf2_eigen.h>

namespace robot_application
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("point_to_point");
using namespace std::chrono_literals;

PointToPoinTask::PointToPoinTask(const rclcpp::Node::SharedPtr& node,
                                 const PointToPoinTask::Parameters& parameters)
  : node_(node), param_task_(parameters)
{
  parameters_comp_path_skill.loadParameters(node_);

  // Initialize MoveItCpp API
  moveit_cpp::MoveItCpp::Options moveit_cpp_options(node_);
  moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(node_, moveit_cpp_options);

  initSkills();
}

void PointToPoinTask::initSkills()
{
  comp_path_skill = std::make_shared<robot_skills::ComputePathWithMoveItCppSkill>(
      node_, parameters_comp_path_skill, moveit_cpp_ptr_);
  exec_traj_skill = std::make_shared<robot_skills::ExecuteTrajectorySkill>(node_, moveit_cpp_ptr_);

  RCLCPP_INFO(LOGGER, "Initial skills");
}

void PointToPoinTask::executeTask()
{
  bool step_success = false;

  /**************************
   * Move To Initial Pose *
   **************************/
  step_success = comp_path_skill->computePath(param_task_.plan_group, param_task_.start_state,
                                              robot_trajectory_);
  if (step_success)
  {
    trajectory_msg_ = moveit_msgs::msg::RobotTrajectory();
    robot_trajectory_->getRobotTrajectoryMsg(trajectory_msg_);
    RCLCPP_INFO(LOGGER, "-------------- Move to %s, Trajectory size: %ld --------------",
                param_task_.start_state.c_str(), trajectory_msg_.joint_trajectory.points.size());
    step_success = exec_traj_skill->execute_trajectory(param_task_.plan_group, robot_trajectory_);
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
    RCLCPP_INFO(
        LOGGER,
        "\n----------------------------\n Move to Point [%d]: \n ----------------------------", i);
    if (i == 0)
    {
      point_msg.point = pose_array.poses.at(i).position;
      point_msg.point.z = point_msg.point.z + param_task_.offset;
      point_msg.header.frame_id = frame_id;

      trajectory_msg_ = moveit_msgs::msg::RobotTrajectory();
      comp_path_skill->setPlanner("pilz_industrial_motion_planner", "LIN");
      step_success = comp_path_skill->computePath(param_task_.plan_group, point_msg,
                                                  robot_trajectory_, param_task_.ik_frame);

      if (step_success)
      {
        trajectory_msg_ = moveit_msgs::msg::RobotTrajectory();
        robot_trajectory_->getRobotTrajectoryMsg(trajectory_msg_);
        RCLCPP_DEBUG(
            LOGGER,
            "\n----------------------------\n Trajectory size:%ld \n ----------------------------",
            trajectory_msg_.joint_trajectory.points.size());
        step_success =
            exec_traj_skill->execute_trajectory(param_task_.plan_group, robot_trajectory_);
      }
    }
    else
    {
      auto direction = geometry_msgs::msg::Vector3();
      direction.x = pose_array.poses.at(i).position.x - pose_array.poses.at(i - 1).position.x;
      direction.y = pose_array.poses.at(i).position.y - pose_array.poses.at(i - 1).position.y;
      direction.z = pose_array.poses.at(i).position.z - pose_array.poses.at(i - 1).position.z;

      std::vector<robot_trajectory::RobotTrajectoryPtr> robot_trajectories;
      step_success = comp_path_skill->computeRelative(param_task_.plan_group, direction,
                                                      robot_trajectories, param_task_.ik_frame);
      if (step_success)
      {
        for (const auto& traj : robot_trajectories)
        {
          trajectory_msg_ = moveit_msgs::msg::RobotTrajectory();
          traj->getRobotTrajectoryMsg(trajectory_msg_);
          RCLCPP_DEBUG(LOGGER,
                       "\n----------------------------\n Trajectory size:%ld \n "
                       "----------------------------",
                       trajectory_msg_.joint_trajectory.points.size());
          step_success = exec_traj_skill->execute_trajectory(param_task_.plan_group, traj);
        }
      }
    }

    rclcpp::sleep_for(1s);
  }
}

}  // namespace robot_application
