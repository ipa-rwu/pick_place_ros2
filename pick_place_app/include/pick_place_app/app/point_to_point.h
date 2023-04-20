#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pick_place_app/skills/compute_path_with_moveitcpp_skill.h"
#include "pick_place_app/skills/execute_trajectory_skill.h"
#include "pick_place_app/skills/modify_planning_scene_skill.h"

namespace robot_application
{
class PointToPoinTask
{
public:
  struct Parameters
  {
    std::string plan_group;
    std::string waypoints_file;
    std::string start_state;
    std::string ik_frame;
    double offset;
    std::vector<double> box_size;

    void loadParameters(const rclcpp::Node::SharedPtr& node)
    {
      node->get_parameter("plan_group", plan_group);
      node->get_parameter("waypoints_file", waypoints_file);
      node->get_parameter("start_state", start_state);
      node->get_parameter_or("ik_frame", ik_frame, std::string(""));
      node->get_parameter_or("offset", offset, 0.0);
      node->get_parameter_or("box_size", box_size, std::vector<double>({ 0.017, 0.017, 0.017 }));
    }
  };

  PointToPoinTask(const rclcpp::Node::SharedPtr& node, const Parameters& parameters);
  void initSkills();
  void executeTask();

protected:
  robot_skills::ComputePathWithMoveItCppSkill::SharedPtr comp_path_skill;
  robot_skills::ComputePathWithMoveItCppSkill::Parameters parameters_comp_path_skill;

  robot_skills::ExecuteTrajectorySkill::SharedPtr exec_traj_skill;

private:
  PointToPoinTask::Parameters param_task_;
  rclcpp::Node::SharedPtr node_;
  robot_trajectory::RobotTrajectoryPtr robot_trajectory_;
  moveit_msgs::msg::RobotTrajectory trajectory_msg_;
  moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
};

}  // namespace robot_application
