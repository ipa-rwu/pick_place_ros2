#ifndef ROBOT_SKILLS__COMPUTE_PATH_WITH_MOVEITCPP_SKILL_H_
#define ROBOT_SKILLS__COMPUTE_PATH_WITH_MOVEITCPP_SKILL_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <boost/any.hpp>

#include <memory>

namespace robot_skills
{
class ComputePathWithMoveItCppSkill
{
public:
  /**
   * @brief
   *
   */
  RCLCPP_SMART_PTR_DEFINITIONS(ComputePathWithMoveItCppSkill)

  struct Parameters
  {
    std::string planner_id;
    double planning_time;
    int planning_attempts;
    double goal_position_tolerance;     // 0.1 mm
    double goal_orientation_tolerance;  // ~0.1 deg
    double goal_joint_tolerance;
    double step_size;
    double jump_threshold;
    double max_velocity_scaling_factor;
    double max_acceleration_scaling_factor;
    double min_fraction;
    std::string planning_pipeline;
    std::string GET_PLANNING_SCENE_SERVICE_NAME;
    std::string PILZ_PLANNING_PIPELINE = "pilz_industrial_motion_planner";

    void loadParameters(const rclcpp::Node::SharedPtr& node)
    {
      std::string ns = "compute_path_moveitcpp_skill.";
      node->get_parameter_or(ns + "planner_id", planner_id, std::string(""));
      node->get_parameter_or(ns + "planning_pipeline", planning_pipeline, std::string(""));
      node->get_parameter_or(ns + "planning_time", planning_time, 5.0);
      node->get_parameter_or(ns + "planning_attempts", planning_attempts, 5);
      node->get_parameter_or(ns + "max_velocity_scaling_factor", max_velocity_scaling_factor, 0.5);
      node->get_parameter_or(ns + "max_acceleration_scaling_factor",
                             max_acceleration_scaling_factor, 0.5);
      node->get_parameter_or(ns + "min_fraction", min_fraction, 0.7);
      node->get_parameter_or(ns + "jump_threshold", jump_threshold, 0.0);
      node->get_parameter_or(ns + "step_size", step_size, 0.005);
      node->get_parameter_or(ns + "goal_joint_tolerance", goal_joint_tolerance, 1e-4);
      node->get_parameter_or(ns + "goal_orientation_tolerance", goal_orientation_tolerance, 1e-3);
      node->get_parameter_or(ns + "goal_position_tolerance", goal_position_tolerance, 1e-4);
      node->get_parameter_or(ns + "GET_PLANNING_SCENE_SERVICE_NAME",
                             GET_PLANNING_SCENE_SERVICE_NAME,
                             std::string("compute_path_moveitcpp_skill/get_planning_scene"));
    }
  };

  ComputePathWithMoveItCppSkill(rclcpp::Node::SharedPtr node, const Parameters& parameters,
                                moveit_cpp::MoveItCppPtr& moveit_cpp_ptr);

  ~ComputePathWithMoveItCppSkill();

  moveit::core::RobotState getRobotStartState(std::vector<std::string> joint_names,
                                              std::vector<double> joint_state);

  /**
   * @brief Plan from current scene to target RobotState
   *
   * @param target_robot_state
   * @param jmg
   * @param result
   * @param path_constraints
   * @return true
   * @return false
   */
  bool plan(const moveit::core::RobotState& target_robot_state,
            const moveit::core::JointModelGroup* jmg, robot_trajectory::RobotTrajectoryPtr& result,
            const moveit_msgs::msg::Constraints& path_constraints = moveit_msgs::msg::Constraints());

  /**
   * @brief Plan from current scene to a Isometry3d in planning frame
   *
   * @param link
   * @param offset
   * @param target_eigen
   * @param jmg
   * @param result
   * @param path_constraints
   * @return true
   * @return false
   */
  bool plan(const moveit::core::LinkModel& link, const Eigen::Isometry3d& offset,
            const Eigen::Isometry3d& target_eigen, const moveit::core::JointModelGroup* jmg,
            robot_trajectory::RobotTrajectoryPtr& result,
            const moveit_msgs::msg::Constraints& path_constraints = moveit_msgs::msg::Constraints());

  bool planCartesianToPose(const std::string& group,
                           const moveit::core::RobotState& current_robot_state,
                           const moveit::core::LinkModel& link, const Eigen::Isometry3d& offset,
                           const Eigen::Isometry3d& target_eigen,
                           robot_trajectory::RobotTrajectoryPtr& result);

  /**
   * @brief Planning Cartesian path by following waypoints
   *
   * @param group
   * @param waypoints pose of waypoint in global(planning) frame
   * @param result
   * @return true
   * @return false
   */
  bool planCartesianMoveGroup(const std::string& group,
                              const std::vector<geometry_msgs::msg::Pose>& waypoints,
                              robot_trajectory::RobotTrajectoryPtr& result);

  bool computePath(
      const std::string& group, const boost::any& goal,
      robot_trajectory::RobotTrajectoryPtr& robot_trajectory, const std::string& ik_frame_id = "",
      bool compute_cartesian_path = false,
      const moveit_msgs::msg::Constraints& path_constraints = moveit_msgs::msg::Constraints());

  bool computeRelative(const std::string& group, geometry_msgs::msg::Vector3 direction,
                       std::vector<robot_trajectory::RobotTrajectoryPtr>& robot_trajectories,
                       const std::string& ik_frame_id = "");

  bool checkCollision(const planning_scene::PlanningSceneConstPtr& current_scene);

  void setPlanner(const std::string& planning_pipeline = "", const std::string& planner_id = "");

protected:
  /**
   * @brief initial a Planning parameter
   *
   * @param plan_params
   */
  void
  initPlanComponentParameters(moveit_cpp::PlanningComponent::PlanRequestParameters& plan_params);

  /**
   * @brief Check goal type, transfer into moveit::core::RobotState
   *
   * @param goal can be std::string named joint state; moveit_msgs::msg::RobotState, joint map
   * @param jmg moveit::core::JointModelGroup
   * @param target_state result: moveit::core::RobotState
   * @return true
   * @return false
   */
  bool getJointStateGoal(const boost::any& goal, const moveit::core::JointModelGroup* jmg,
                         moveit::core::RobotState& state);

  /**
   * @brief Transform the pose into the planning frame, get Eigen::Isometry3d
   *
   * @param goal if it is geometry_msgs::msg::PoseStamped
   * @param scene provide planning frame
   * @param target the goal transformed into the planning frame
   * @return true
   * @return false
   */
  bool getPoseGoal(const boost::any& goal, const planning_scene::PlanningSceneConstPtr& scene,
                   Eigen::Isometry3d& target);

  /**
   * @brief Transform the PointStamped into the planning frame, keep link orientation, get Eigen::Isometry3d
   *
   * @param goal if it is a PointStamped (x,y,z) with reference coordinate frame
   * @param ik_pose provide link orientation
   * @param scene provide planning frame
   * @param target_eigen Eigen::Isometry3d
   * @return true
   * @return false
   */
  bool getPointGoal(const boost::any& goal, const Eigen::Isometry3d& ik_pose,
                    const planning_scene::PlanningSceneConstPtr& scene,
                    Eigen::Isometry3d& target_eigen);

protected:
private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  ComputePathWithMoveItCppSkill::Parameters parameters_;
  moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
  moveit_cpp::PlanningComponentPtr planning_component_ptr_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  trajectory_processing::TimeOptimalTrajectoryGeneration time_parametrization_;
  moveit_cpp::PlanningComponent::PlanRequestParameters plan_params_;
  bool set_planner_called_;
};

}  // namespace robot_skills

#endif
