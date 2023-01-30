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
    std::string planning_plugin = "ompl_interface/OMPLPlanner";
    double planning_timeout = 5.0;
    int num_planning_attempts = 3;
    std::string ik_frame;
    double goal_position_tolerance = 0.1;
    double goal_orientation_tolerance = 0.1;
    double goal_joint_tolerance = 0.1;
    double step_size = 0.005;
    double jump_threshold = 1.5;
    double max_velocity_scaling_factor = 1.0;
    double max_acceleration_scaling_factor = 1.0;

    void loadParameters(const rclcpp::Node::SharedPtr& node);
  };

  ComputePathSkill(rclcpp::Node::SharedPtr node, const Parameters& parameters,
                   moveit::core::RobotModelPtr robot_model,
                   robot_model_loader::RobotModelLoaderPtr robot_model_loader);

  ~ComputePathSkill();

  moveit::core::RobotState getRobotStartState(std::vector<std::string> joint_names,
                                              std::vector<double> joint_state);

  void initMotionPlanRequest(moveit_msgs::msg::MotionPlanRequest& req,
                             const moveit::core::JointModelGroup* jmg, double timeout);

  bool plan(const moveit::core::LinkModel& link, const Eigen::Isometry3d& offset,
            const Eigen::Isometry3d& target, const moveit::core::JointModelGroup* jmg,
            double timeout, robot_trajectory::RobotTrajectoryPtr& result,
            const moveit_msgs::msg::Constraints& path_constraints = moveit_msgs::msg::Constraints());

  bool plan(const planning_scene::PlanningSceneConstPtr& target_scene,
            const moveit::core::JointModelGroup* jmg, double timeout,
            robot_trajectory::RobotTrajectoryPtr& result,
            const moveit_msgs::msg::Constraints& path_constraints);

  bool planCartesian(const std::string& group, const moveit::core::RobotState& current_robot_state,
                     const std::string& plan_frame_id, const moveit::core::LinkModel& link,
                     const Eigen::Isometry3d& offset, const Eigen::Isometry3d& target_eigen,
                     robot_trajectory::RobotTrajectoryPtr& result);

  bool compute(planning_scene::PlanningScenePtr& scene, const std::string& group,
               const boost::any& goal, robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
               const moveit_msgs::msg::Constraints& path_constraints, bool compute_cartesian_path);

  planning_pipeline::PlanningPipelinePtr planning_pipeline_;

protected:
  bool getJointStateGoal(const boost::any& goal, const moveit::core::JointModelGroup* jmg,
                         moveit::core::RobotState& state);

  bool getPoseGoal(const boost::any& goal, const planning_scene::PlanningScenePtr& scene,
                   Eigen::Isometry3d& target);

  bool getPointGoal(const boost::any& goal, const Eigen::Isometry3d& ik_pose,
                    const planning_scene::PlanningScenePtr& scene, Eigen::Isometry3d& target_eigen);

protected:
  moveit::planning_interface::MoveGroupInterfacePtr move_group;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr traj_publisher_;
  ComputePathSkill::Parameters parameters_;
  moveit::core::RobotModelPtr robot_model_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  trajectory_processing::TimeOptimalTrajectoryGeneration time_parametrization_;
};

}  // namespace robot_skills

#endif
