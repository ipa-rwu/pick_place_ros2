#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pick_place_app/skills/compute_path_skill.h"
#include "pick_place_app/skills/execute_trajectory_skill.h"
#include "pick_place_app/skills/io_gripper_with_ur_skill.h"

namespace robot_application
{
class PickPlaceTask
{
public:
  struct Parameters
  {
    std::string arm_group_name;
    geometry_msgs::msg::Pose object_pose;
    geometry_msgs::msg::Pose place_pose;
    std::string object_frame_id;
    std::string place_frame_id;
    std::string object_name;
    std::string detect_state_name;
    double pick_offset;

    void loadParameters(const rclcpp::Node::SharedPtr& node);
  };

  PickPlaceTask(const rclcpp::Node::SharedPtr& node, const Parameters& parameters);
  void initSkills();
  void loadRobot();
  void executeTask();

protected:
  robot_skills::ComputePathSkill::SharedPtr comp_path_skill;
  robot_skills::ComputePathSkill::Parameters parameters_comp_path_skill;

  robot_skills::ExecuteTrajectorySkill::SharedPtr exec_traj_skill;

  robot_skills::IOGripperWithURSkill::SharedPtr io_gripper_skill;
  robot_skills::IOGripperWithURSkill::Parameters parameters_io_gripper_skill;

  robot_trajectory::RobotTrajectoryPtr robot_trajectory;
  moveit_msgs::msg::RobotTrajectory trajectory_msg;

private:
  rclcpp::Node::SharedPtr node_;
  planning_scene::PlanningScenePtr planning_scene_;
  moveit::core::RobotModelPtr robot_model_;
  PickPlaceTask::Parameters pick_place_task_param_;
  robot_model_loader::RobotModelLoaderPtr rm_loader_;
  moveit::core::RobotStatePtr current_robot_state_;
};

}  // namespace robot_application
