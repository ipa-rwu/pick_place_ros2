#pragma once

#include <rclcpp/rclcpp.hpp>

#include "pick_place_app/skills/compute_path_skill.h"
#include "pick_place_app/skills/execute_trajectory_skill.h"
#include "pick_place_app/skills/io_gripper_with_ur_skill.h"
#include "pick_place_app/skills/modify_planning_scene_skill.h"
#include "pick_place_app/skills/detect_aruco_marker_skill.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
    std::vector<double> box_size;
    double lift_distance;
    double place_offset;
    std::string path_constraints_file;
    double retreat_offset;

    void loadParameters(const rclcpp::Node::SharedPtr& node);
  };

  PickPlaceStaticTask(const rclcpp::Node::SharedPtr& node, const Parameters& parameters);
  void initSkills();
  void loadRobot();
  void executeTask();

protected:
  robot_skills::ComputePathSkill::SharedPtr comp_path_skill;
  robot_skills::ComputePathSkill::Parameters parameters_comp_path_skill;

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
  moveit::core::RobotModelPtr robot_model_;
  PickPlaceStaticTask::Parameters param_pick_place_static_;
  robot_model_loader::RobotModelLoaderPtr rm_loader_;
  moveit::planning_interface::PlanningSceneInterfacePtr psi_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};

}  // namespace robot_application
