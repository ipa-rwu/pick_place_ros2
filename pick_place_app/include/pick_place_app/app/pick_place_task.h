#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pick_place_app/skills/compute_path_skill.h"
#include "pick_place_app/skills/execute_trajectory_skill.h"
#include "pick_place_app/skills/io_gripper_with_ur_skill.h"
#include "pick_place_app/skills/modify_planning_scene_skill.h"
#include "pick_place_app/skills/detect_aruco_marker_skill.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace robot_application
{
class PickPlaceTask
{
public:
  struct Parameters
  {
    std::string arm_group_name;
    std::string detect_state_name;
    double pick_offset;
    std::vector<double> box_size;
    double lift_distance;
    double place_offset;
    int object_marker_id;
    int place_marker_id;
    std::string path_constraints_file;
    double retreat_offset;

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

  robot_skills::ModifyPlanningSceneSkill::SharedPtr modify_planning_scene_skill;

  robot_skills::DetectArucoMarkerSkill::SharedPtr detect_aruco_marker_skill;
  robot_skills::DetectArucoMarkerSkill::Parameters parameters_detect_aruco_marker_skill;

  robot_trajectory::RobotTrajectoryPtr robot_trajectory;
  moveit_msgs::msg::RobotTrajectory trajectory_msg;

private:
  rclcpp::Node::SharedPtr node_;
  planning_scene::PlanningScenePtr planning_scene_;
  moveit::core::RobotModelPtr robot_model_;
  PickPlaceTask::Parameters pick_place_task_param_;
  robot_model_loader::RobotModelLoaderPtr rm_loader_;
  moveit::core::RobotStatePtr current_robot_state_;
  moveit::planning_interface::PlanningSceneInterfacePtr psi_;
  geometry_msgs::msg::PoseStamped object_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr object_pose_ptr_;
  geometry_msgs::msg::PoseStamped::SharedPtr place_pose_ptr_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};

}  // namespace robot_application
