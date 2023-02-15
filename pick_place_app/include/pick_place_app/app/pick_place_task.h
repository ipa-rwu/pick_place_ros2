#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pick_place_app/skills/compute_path_with_moveitcpp_skill.h"
#include "pick_place_app/skills/execute_trajectory_skill.h"
#include "pick_place_app/skills/io_gripper_with_ur_skill.h"
#include "pick_place_app/skills/modify_planning_scene_skill.h"
#include "pick_place_app/skills/detect_aruco_marker_skill.h"

namespace robot_application
{
class PickPlaceTask
{
public:
  struct Parameters
  {
    std::string arm_group_name;
    std::string detect_state_name;
    std::vector<double> box_size;
    std::vector<double> container_size;
    std::vector<double> pre_pick_offset;
    std::vector<double> pick_offset;
    std::vector<double> pre_place_offset;
    std::vector<double> place_offset;
    std::vector<double> retreat_offset;
    int object_marker_id;
    int place_marker_id;
    std::string path_constraints_file;
    std::string ik_frame;

    void loadParameters(const rclcpp::Node::SharedPtr& node)
    {
      node->get_parameter("arm_group_name", arm_group_name);
      node->get_parameter("detect_state_name", detect_state_name);
      node->get_parameter("pre_pick_offset", pre_pick_offset);
      node->get_parameter("pick_offset", pick_offset);
      node->get_parameter("pre_place_offset", pre_place_offset);
      node->get_parameter("place_offset", place_offset);
      node->get_parameter("retreat_offset", retreat_offset);
      node->get_parameter("place_marker_id", place_marker_id);
      node->get_parameter("object_marker_id", object_marker_id);

      node->get_parameter_or("path_constraints_file", path_constraints_file, std::string(""));
      node->get_parameter_or("ik_frame", ik_frame, std::string(""));
      node->get_parameter_or("box_size", box_size, std::vector<double>({ 0.017, 0.017, 0.017 }));
      node->get_parameter_or("container_size", container_size,
                             std::vector<double>({ 0.06, 0.07, 0.006 }));
    }
  };

  PickPlaceTask(const rclcpp::Node::SharedPtr& node, const Parameters& parameters);
  void initSkills();
  void executeTask();

protected:
  robot_skills::ComputePathWithMoveItCppSkill::SharedPtr comp_path_skill;
  robot_skills::ComputePathWithMoveItCppSkill::Parameters parameters_comp_path_skill;

  robot_skills::ExecuteTrajectorySkill::SharedPtr exec_traj_skill;

  robot_skills::IOGripperWithURSkill::SharedPtr io_gripper_skill;
  robot_skills::IOGripperWithURSkill::Parameters parameters_io_gripper_skill;

  robot_skills::ModifyPlanningSceneSkill::SharedPtr modify_planning_scene_skill;

  robot_skills::DetectArucoMarkerSkill::SharedPtr detect_aruco_marker_skill;
  robot_skills::DetectArucoMarkerSkill::Parameters parameters_detect_aruco_marker_skill;

  robot_trajectory::RobotTrajectoryPtr robot_trajectory;
  moveit_msgs::msg::RobotTrajectory trajectory_msg;

private:
  rclcpp::Node::SharedPtr node_;
  PickPlaceTask::Parameters param_pick_place_;
  geometry_msgs::msg::PoseStamped object_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr object_pose_ptr_;
  geometry_msgs::msg::PoseStamped::SharedPtr place_pose_ptr_;
  moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
};

}  // namespace robot_application
