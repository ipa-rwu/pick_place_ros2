#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pick_place_app/skills/compute_path_skill.h"
#include "pick_place_app/skills/execute_trajectory_skill.h"
#include "pick_place_app/skills/modify_planning_scene_skill.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace robot_application
{
class PointToPoinTask
{
public:
  struct Parameters
  {
    std::string arm_group_name;
    std::string arm_hand_group_name;
    std::string waypoints_file;
    std::string start_state;
    std::string ik_frame;
    double offset;
    std::vector<double> box_size;

    void loadParameters(const rclcpp::Node::SharedPtr& node);
  };

  PointToPoinTask(const rclcpp::Node::SharedPtr& node, const Parameters& parameters);
  void initSkills();
  void loadRobot();
  void executeTask();

protected:
  robot_skills::ComputePathSkill::SharedPtr comp_path_skill;
  robot_skills::ComputePathSkill::Parameters parameters_comp_path_skill;

  robot_skills::ExecuteTrajectorySkill::SharedPtr exec_traj_skill;

  robot_skills::ModifyPlanningSceneSkill::SharedPtr modify_planning_scene_skill;

private:
  moveit::core::RobotModelPtr robot_model_;
  PointToPoinTask::Parameters param_task_;
  robot_model_loader::RobotModelLoaderPtr rm_loader_;
  moveit::planning_interface::PlanningSceneInterfacePtr psi_;

  rclcpp::Node::SharedPtr node_;
  robot_trajectory::RobotTrajectoryPtr robot_trajectory_;
  moveit_msgs::msg::RobotTrajectory trajectory_msg_;
};

}  // namespace robot_application
