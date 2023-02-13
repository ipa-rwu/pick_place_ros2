#ifndef ROBOT_SKILLS__COMPUTE_PATH_SKILL_H_
#define ROBOT_SKILLS__COMPUTE_PATH_SKILL_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/time_parameterization.h>

#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <boost/any.hpp>

#include <memory>

namespace robot_skills
{
class ComputePathSkill
{
public:
  /**
   * @brief
   *
   */
  RCLCPP_SMART_PTR_DEFINITIONS(ComputePathSkill)

  struct Parameters
  {
    std::string planner_id =
        "ompl_interface/OMPLPlanner";  // specific planner id in a planning pipeline
    double planning_timeout = 10.0;
    int num_planning_attempts = 10;
    std::string ik_frame;
    double goal_position_tolerance = 1e-4;     // 0.1 mm
    double goal_orientation_tolerance = 1e-3;  // ~0.1 deg
    double goal_joint_tolerance = 1e-4;
    double step_size = 0.005;
    double jump_threshold = 0.0;
    double max_velocity_scaling_factor = 0.5;
    double max_acceleration_scaling_factor = 0.5;
    double min_fraction = 0.7;
    std::string planning_plugin;
    void loadParameters(const rclcpp::Node::SharedPtr& node);
  };

  struct Specification
  {
    moveit::core::RobotModelConstPtr model;
    std::string ns{ "ompl" };
    std::string pipeline{ "ompl" };
    std::string adapter_param{ "request_adapters" };
  };

  ComputePathSkill(rclcpp::Node::SharedPtr node, const Parameters& parameters,
                   robot_model_loader::RobotModelLoaderPtr robot_model_loader,
                   planning_scene_monitor::PlanningSceneMonitorPtr psm);

  ~ComputePathSkill();

  moveit::core::RobotState getRobotStartState(std::vector<std::string> joint_names,
                                              std::vector<double> joint_state);

  /**
   * @brief Plan from current scene to target RobotState
   *
   * @param current_scene
   * @param target_robot_state
   * @param jmg
   * @param timeout
   * @param result
   * @param path_constraints
   * @return true
   * @return false
   */
  bool
  plan(const moveit::core::RobotState& target_robot_state, const moveit::core::JointModelGroup* jmg,
       double timeout, robot_trajectory::RobotTrajectoryPtr& result,
       const moveit_msgs::msg::Constraints& path_constraints = moveit_msgs::msg::Constraints());

  /**
   * @brief Plan from current scene to a Isometry3d in planning frame
   *
   * @param link
   * @param offset
   * @param target
   * @param jmg
   * @param timeout
   * @param result
   * @param path_constraints
   * @return true
   * @return false
   */
  bool plan(const moveit::core::LinkModel& link, const Eigen::Isometry3d& offset,
            const Eigen::Isometry3d& target_eigen, const moveit::core::JointModelGroup* jmg,
            double timeout, robot_trajectory::RobotTrajectoryPtr& result,
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
   * @param current_robot_state
   * @param result
   * @return true
   * @return false
   */
  bool planCartesian(const std::string& group,
                     const std::vector<geometry_msgs::msg::Pose>& waypoints,
                     const moveit::core::RobotState& current_robot_state,
                     robot_trajectory::RobotTrajectoryPtr& result);

  bool computePath(
      const std::string& group, const boost::any& goal,
      robot_trajectory::RobotTrajectoryPtr& robot_trajectory, const std::string& ik_frame_id = "",
      bool compute_cartesian_path = false,
      const moveit_msgs::msg::Constraints& path_constraints = moveit_msgs::msg::Constraints());

  bool planRelativeCartesian(moveit::core::RobotState& current_robot_state,
                             const std::string& group, const moveit::core::LinkModel& link,
                             geometry_msgs::msg::Vector3 direction,
                             robot_trajectory::RobotTrajectoryPtr& robot_trajectory);

  bool computeRelative(const std::string& group, geometry_msgs::msg::Vector3 direction,
                       robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
                       const std::string& ik_frame_id = "");

  bool checkCollision(const planning_scene::PlanningSceneConstPtr& current_scene);

  planning_pipeline::PlanningPipelinePtr planning_pipeline_;

protected:
  /**
   * @brief initial a motion plan request
   *
   * @param req moveit_msgs::msg::MotionPlanRequest
   * @param jmg target joint model group for planning
   * @param timeout
   */
  void initMotionPlanRequest(moveit_msgs::msg::MotionPlanRequest& req,
                             const moveit::core::JointModelGroup* jmg, double timeout);

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

  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr traj_publisher_;
  ComputePathSkill::Parameters parameters_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  trajectory_processing::TimeOptimalTrajectoryGeneration time_parametrization_;
};

}  // namespace robot_skills

#endif
