#pragma once

#include <rclcpp/rclcpp.hpp>

#include "pick_place_app/skills/compute_path_with_moveitcpp_skill.h"
#include "pick_place_app/skills/execute_trajectory_skill.h"
#include "pick_place_app/skills/io_gripper_with_ur_skill.h"
#include "pick_place_app/skills/modify_planning_scene_skill.h"
#include "pick_place_app/skills/detect_aruco_marker_skill.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace robot_application
{
class PickPlaceStaticTask
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
    double lift_distance;
    double place_offset;
    std::string path_constraints_file;
    double retreat_offset;
    std::string ik_frame;
    std::vector<double> box_size;

    void loadParameters(const rclcpp::Node::SharedPtr& node)
    {
      node->get_parameter("arm_group_name", arm_group_name);
      node->get_parameter("place_frame_id", place_frame_id);
      node->get_parameter("object_frame_id", object_frame_id);
      node->get_parameter("pick_offset", pick_offset);
      node->get_parameter("lift_distance", lift_distance);
      node->get_parameter("place_offset", place_offset);
      node->get_parameter("retreat_offset", retreat_offset);
      node->get_parameter("detect_state_name", detect_state_name);

      node->get_parameter_or("path_constraints_file", path_constraints_file, std::string(""));
      node->get_parameter_or("ik_frame", ik_frame, std::string(""));
      node->get_parameter_or("object_name", object_name, std::string("box"));
      node->get_parameter_or("box_size", box_size, std::vector<double>({ 0.017, 0.017, 0.017 }));

      size_t errors = 0;
      errors += !rosparam_shortcuts::get(node, "object_pose", object_pose);
      errors += !rosparam_shortcuts::get(node, "place_pose", place_pose);
      rosparam_shortcuts::shutdownIfError(errors);
    }
  };

  PickPlaceStaticTask(const rclcpp::Node::SharedPtr& node, const Parameters& parameters);
  void initSkills();
  void loadRobot();
  void executeTask();

protected:
  robot_skills::ComputePathWithMoveItCppSkill::SharedPtr comp_path_skill;
  robot_skills::ComputePathWithMoveItCppSkill::Parameters parameters_comp_path_skill;

  robot_skills::ExecuteTrajectorySkill::SharedPtr exec_traj_skill;

  robot_skills::IOGripperWithURSkill::SharedPtr io_gripper_skill;
  robot_skills::IOGripperWithURSkill::Parameters parameters_io_gripper_skill;

  robot_skills::ModifyPlanningSceneSkill::SharedPtr modify_planning_scene_skill;

  robot_trajectory::RobotTrajectoryPtr robot_trajectory;
  moveit_msgs::msg::RobotTrajectory trajectory_msg;

private:
  rclcpp::Node::SharedPtr node_;
  PickPlaceStaticTask::Parameters param_pick_place_static_;
  moveit::planning_interface::PlanningSceneInterfacePtr psi_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
};

}  // namespace robot_application
