#include "pick_place_app/skills/compute_path_skill.h"
#include "pick_place_app/skills/utils.h"

#include <moveit/kinematic_constraints/utils.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace robot_skills
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("compute_path_skill");

void ComputePathSkill::Parameters::loadParameters(const rclcpp::Node::SharedPtr& node)
{
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(node, "ik_frame",
                                     ik_frame);  //

  errors += !rosparam_shortcuts::get(node, "min_fraction", min_fraction);  //
  errors += !rosparam_shortcuts::get(node, "max_acceleration_scaling_factor",
                                     max_acceleration_scaling_factor);  //

  errors += !rosparam_shortcuts::get(node, "max_velocity_scaling_factor",
                                     max_velocity_scaling_factor);  //

  rosparam_shortcuts::shutdownIfError(errors);

  // Optional parameters (default value exists => no shutdown required if
  // loading fails)
  size_t warnings = 0;
  warnings += !rosparam_shortcuts::get(node, "planning_timeout",
                                       planning_timeout);  // e.g. 1000s
  warnings += !rosparam_shortcuts::get(node, "num_planning_attempts",
                                       num_planning_attempts);  // e.g. 3

  if (warnings > 0)
    RCLCPP_WARN(LOGGER, "Failed to load optional parameters.");
}

ComputePathSkill::ComputePathSkill(rclcpp::Node::SharedPtr node,
                                   const ComputePathSkill::Parameters& parameters,
                                   moveit::core::RobotModelPtr robot_model,
                                   robot_model_loader::RobotModelLoaderPtr robot_model_loader)
  : node_(node)
  , parameters_(parameters)
  , robot_model_(robot_model)
  , robot_model_loader_(robot_model_loader)
{
  psm_.reset(new planning_scene_monitor::PlanningSceneMonitor(node_, robot_model_loader_));

  /* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
                       the internal planning scene accordingly */
  psm_->startSceneMonitor();
  /* listens to changes of world geometry, collision objects, and (optionally) octomaps
                                world geometry, collision objects and optionally octomaps */
  psm_->startWorldGeometryMonitor();
  /* listen to joint state updates as well as changes in attached collision objects
                        and update the internal planning scene accordingly*/
  psm_->startStateMonitor();

  /* Set up a PlanningPipeline, will use it generate plan */
  planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(
      robot_model_, node_, "", "planning_plugin", "request_adapters"));
}

ComputePathSkill::~ComputePathSkill() = default;

void ComputePathSkill::initMotionPlanRequest(moveit_msgs::msg::MotionPlanRequest& req,
                                             const moveit::core::JointModelGroup* jmg,
                                             double timeout)
{
  req.group_name = jmg->getName();
  req.pipeline_id = parameters_.planning_plugin;
  req.planner_id = parameters_.planner_id;
  req.allowed_planning_time = timeout;
  req.start_state.is_diff = true;  // we don't specify an extra start state
  req.max_velocity_scaling_factor = parameters_.max_velocity_scaling_factor;
  req.max_velocity_scaling_factor = parameters_.max_acceleration_scaling_factor;
  req.num_planning_attempts = parameters_.num_planning_attempts;
}

bool ComputePathSkill::getJointStateGoal(const boost::any& goal,
                                         const moveit::core::JointModelGroup* jmg,
                                         moveit::core::RobotState& target_state)
{
  try
  {
    // try named joint pose
    const std::string& named_joint_pose = boost::any_cast<std::string>(goal);
    if (!target_state.setToDefaultValues(jmg, named_joint_pose))
      RCLCPP_ERROR(LOGGER, "Unknown joint pose: %s", named_joint_pose.c_str());
    RCLCPP_DEBUG(LOGGER, "Get named_joint_pose: %s", named_joint_pose.c_str());
    target_state.update();
    return true;
  }
  catch (const boost::bad_any_cast&)
  {
  }

  try
  {
    // try RobotState
    const moveit_msgs::msg::RobotState& msg = boost::any_cast<moveit_msgs::msg::RobotState>(goal);
    if (!msg.is_diff)
      RCLCPP_ERROR(LOGGER, "Expecting a diff state.");
    // validate specified joints
    const auto& accepted = jmg->getJointModelNames();
    for (const auto& name : msg.joint_state.name)
      if (std::find(accepted.begin(), accepted.end(), name) == accepted.end())
        RCLCPP_ERROR(LOGGER, "Joint '%s' is not part of group '%s'", name.c_str(),
                     jmg->getName().c_str());
    for (const auto& name : msg.multi_dof_joint_state.joint_names)
      if (std::find(accepted.begin(), accepted.end(), name) == accepted.end())
        RCLCPP_ERROR(LOGGER, "Joint '%s' is not part of group '%s'", name.c_str(),
                     jmg->getName().c_str());

    moveit::core::robotStateMsgToRobotState(msg, target_state, false);
    return true;
  }
  catch (const boost::bad_any_cast&)
  {
  }

  try
  {
    // try joint map
    const std::map<std::string, double>& joint_map =
        boost::any_cast<std::map<std::string, double>>(goal);
    const auto& accepted = jmg->getJointModelNames();
    for (const auto& joint : joint_map)
    {
      if (std::find(accepted.begin(), accepted.end(), joint.first) == accepted.end())
        RCLCPP_ERROR(LOGGER, "Joint '%s' is not part of group '%s'", joint.first.c_str(),
                     jmg->getName().c_str());
      target_state.setVariablePosition(joint.first, joint.second);
    }
    target_state.update();
    return true;
  }
  catch (const boost::bad_any_cast&)
  {
  }
  return false;
}

bool ComputePathSkill::getPoseGoal(const boost::any& goal,
                                   const planning_scene::PlanningSceneConstPtr& scene,
                                   Eigen::Isometry3d& target)
{
  try
  {
    const geometry_msgs::msg::PoseStamped& msg =
        boost::any_cast<geometry_msgs::msg::PoseStamped>(goal);
    tf2::fromMsg(msg.pose, target);

    // transform target into global(planning) frame
    target = scene->getFrameTransform(msg.header.frame_id) * target;
    geometry_msgs::msg::Pose new_msg;
    new_msg = tf2::toMsg(target);
    RCLCPP_INFO(LOGGER, "Get pose goal: \n origin: %s \n transfer to planning frame %s: %s",
                geometry_msgs::msg::to_yaml(msg).c_str(), scene->getPlanningFrame().c_str(),
                geometry_msgs::msg::to_yaml(new_msg).c_str());
  }
  catch (const boost::bad_any_cast&)
  {
    return false;
  }
  return true;
}

bool ComputePathSkill::getPointGoal(const boost::any& goal, const Eigen::Isometry3d& ik_pose,
                                    const planning_scene::PlanningSceneConstPtr& scene,
                                    Eigen::Isometry3d& target_eigen)
{
  try
  {
    const geometry_msgs::msg::PointStamped& target =
        boost::any_cast<geometry_msgs::msg::PointStamped>(goal);
    Eigen::Vector3d target_point;
    tf2::fromMsg(target.point, target_point);
    // transform target into global(planning) frame
    target_point = scene->getFrameTransform(target.header.frame_id) * target_point;

    // retain link orientation
    target_eigen = ik_pose;
    target_eigen.translation() = target_point;
    RCLCPP_INFO(LOGGER, "Get point goal: \n origin: %s \n transfer to planning frame [%s]: %s",
                geometry_msgs::msg::to_yaml(target).c_str(), scene->getPlanningFrame().c_str(),
                geometry_msgs::msg::to_yaml(tf2::toMsg(target_eigen)).c_str());
  }
  catch (const boost::bad_any_cast&)
  {
    return false;
  }
  return true;
}

bool ComputePathSkill::plan(const planning_scene::PlanningSceneConstPtr& current_scene,
                            const moveit::core::RobotState& target_robot_state,
                            const moveit::core::JointModelGroup* jmg, double timeout,
                            robot_trajectory::RobotTrajectoryPtr& result,
                            const moveit_msgs::msg::Constraints& path_constraints)
{
  moveit_msgs::msg::MotionPlanRequest req;
  initMotionPlanRequest(req, jmg, timeout);

  planning_interface::MotionPlanResponse res;

  req.goal_constraints.resize(1);
  req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
      target_robot_state, jmg, parameters_.goal_joint_tolerance);
  req.path_constraints = path_constraints;

  bool success = planning_pipeline_->generatePlan(current_scene, req, res);
  result = res.trajectory_;
  time_parametrization_.computeTimeStamps(*result, parameters_.max_velocity_scaling_factor,
                                          parameters_.max_acceleration_scaling_factor);
  return success;
}

bool ComputePathSkill::plan(const planning_scene::PlanningSceneConstPtr& current_scene,
                            const moveit::core::LinkModel& link, const Eigen::Isometry3d& offset,
                            const Eigen::Isometry3d& target_eigen,
                            const moveit::core::JointModelGroup* jmg, double timeout,
                            robot_trajectory::RobotTrajectoryPtr& result,
                            const moveit_msgs::msg::Constraints& path_constraints)
{
  moveit_msgs::msg::MotionPlanRequest req;
  initMotionPlanRequest(req, jmg, timeout);

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = current_scene->getPlanningFrame();
  target.pose = tf2::toMsg(target_eigen * offset.inverse());
  RCLCPP_INFO(LOGGER, "Start Plan, goal:\n target: %s\n posestamp: %s", link.getName().c_str(),
              geometry_msgs::msg::to_yaml(target).c_str());

  auto new_jmg = jmg;
  kinematics::KinematicsQueryOptions o;
  o.return_approximate_solution = false;
  auto target_robot_state = current_scene->getCurrentState();
  if (target_robot_state.setFromIK(new_jmg, target.pose, link.getName(), 0.0))
  {
    sensor_msgs::msg::JointState current_joint_state;
    moveit::core::robotStateToJointStateMsg(target_robot_state, current_joint_state);
    RCLCPP_DEBUG(LOGGER, "Get IK, goal joint state: %s",
                 sensor_msgs::msg::to_yaml(current_joint_state).c_str());
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Didn't Get IK");
    // return false;
  }

  req.goal_constraints.resize(1);
  req.goal_constraints[0] =
      kinematic_constraints::constructGoalConstraints(link.getName(), target,
                                                      parameters_.goal_position_tolerance,
                                                      parameters_.goal_orientation_tolerance);
  req.path_constraints = path_constraints;

  planning_interface::MotionPlanResponse res;
  planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
  bool success = planning_pipeline_->generatePlan(lscene, req, res);
  if (success)
  {
    result = res.trajectory_;
    time_parametrization_.computeTimeStamps(*result, parameters_.max_velocity_scaling_factor,
                                            parameters_.max_acceleration_scaling_factor);
  }
  return success;
}

bool ComputePathSkill::planCartesian(const std::string& group,
                                     const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                     const moveit::core::RobotState& current_robot_state,
                                     robot_trajectory::RobotTrajectoryPtr& result)
{
  move_group_.reset();
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group);

  double max_fraction = 0;
  double current_fraction = 0;
  auto robot_trajectory_msg = moveit_msgs::msg::RobotTrajectory();

  int try_plan = 0;
  bool success;

  while (try_plan < parameters_.num_planning_attempts)
  {
    try_plan++;
    moveit_msgs::msg::RobotTrajectory tmp_robot_trajectory;
    current_fraction = move_group_->computeCartesianPath(
        waypoints, parameters_.step_size, parameters_.jump_threshold, tmp_robot_trajectory);
    if (current_fraction > max_fraction)
    {
      max_fraction = current_fraction;
      robot_trajectory_msg = tmp_robot_trajectory;
    }
  }
  result->setRobotTrajectoryMsg(current_robot_state, robot_trajectory_msg);

  if (max_fraction < parameters_.min_fraction)
  {
    success = false;
    RCLCPP_ERROR(LOGGER,
                 "Plan Cartesian path faild: (%.2f%% achieved), lower than allowed fraction: %.2f",
                 max_fraction * 100.00, parameters_.min_fraction * 100.00);
    return success;
  }

  success = true;
  RCLCPP_INFO(LOGGER, "Plan Cartesian path succeed!: (%.2f%% achieved)", max_fraction * 100.00);

  return success;
}

bool ComputePathSkill::planRelativeCartesian(moveit::core::RobotState& current_robot_state,
                                             const std::string& group,
                                             const moveit::core::LinkModel& link,
                                             geometry_msgs::msg::Vector3 direction,
                                             robot_trajectory::RobotTrajectoryPtr& robot_trajectory)
{
  // transfer robot state to a pose of link in global(planning) frame
  geometry_msgs::msg::Pose end_pose = tf2::toMsg(current_robot_state.getGlobalLinkTransform(&link));
  RCLCPP_INFO(LOGGER, "Current pose of %s: %s", link.getName().c_str(),
              geometry_msgs::msg::to_yaml(end_pose).c_str());

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(end_pose);

  if (abs(direction.x) > 0)
  {
    end_pose.position.x = end_pose.position.x + direction.x;
    waypoints.push_back(end_pose);
  }
  if (abs(direction.y) > 0)
  {
    end_pose.position.y = end_pose.position.y + direction.y;
    waypoints.push_back(end_pose);
  }
  if (abs(direction.z) > 0)
  {
    end_pose.position.z = end_pose.position.z + direction.z;
    waypoints.push_back(end_pose);
  }
  for (const auto waypoint : waypoints)
  {
    RCLCPP_INFO(LOGGER, "RelativeCartesian waypoins: %s",
                geometry_msgs::msg::to_yaml(waypoint).c_str());
  }

  // plan to Cartesian target
  bool success;
  success = planCartesian(group, waypoints, current_robot_state, robot_trajectory);
  return success;
}

bool ComputePathSkill::computeRelative(const std::string& group,
                                       geometry_msgs::msg::Vector3 direction,
                                       robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
                                       const std::string& ik_frame_id)
{
  planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
  const moveit::core::RobotModelConstPtr& robot_model = lscene->getRobotModel();
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group);
  moveit::core::RobotState current_robot_state = lscene->getCurrentState();
  moveit::core::RobotState target_robot_state = current_robot_state;
  double timeout = parameters_.planning_timeout;
  bool success;

  if (!jmg)
  {
    RCLCPP_ERROR(LOGGER, "Could not joint group from %s", group.c_str());
    return false;
  }

  const moveit::core::LinkModel* link;
  // the translation vector from link to global(planning) frame
  Eigen::Isometry3d ik_pose_world;
  if (!utils::getRobotTipForFrame(lscene, jmg, link, ik_pose_world, ik_frame_id))
    return false;

  success = planRelativeCartesian(current_robot_state, group, *link, direction, robot_trajectory);
  return success;
}

bool ComputePathSkill::planCartesianToPose(const std::string& group,
                                           const moveit::core::RobotState& current_robot_state,
                                           const moveit::core::LinkModel& link,
                                           const Eigen::Isometry3d& offset,
                                           const Eigen::Isometry3d& target_eigen,
                                           robot_trajectory::RobotTrajectoryPtr& result)
{
  // transfer robot state to a pose of link in global(planning) frame
  geometry_msgs::msg::Pose end_pose = tf2::toMsg(current_robot_state.getGlobalLinkTransform(&link));
  RCLCPP_INFO(LOGGER, "Current pose of %s: %s", link.getName().c_str(),
              geometry_msgs::msg::to_yaml(end_pose).c_str());

  geometry_msgs::msg::Pose target;
  target = tf2::toMsg(target_eigen * offset.inverse());

  RCLCPP_INFO(LOGGER, "target pose: %s", geometry_msgs::msg::to_yaml(target).c_str());

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(end_pose);
  waypoints.push_back(target);

  // plan to Cartesian target
  bool success;
  success = planCartesian(group, waypoints, current_robot_state, result);
  return success;
}

bool ComputePathSkill::computePath(const std::string& group, const boost::any& goal,
                                   robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
                                   const std::string& ik_frame_id, bool compute_cartesian_path,
                                   const moveit_msgs::msg::Constraints& path_constraints)
{
  planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
  const moveit::core::RobotModelConstPtr& robot_model = lscene->getRobotModel();
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group);
  moveit::core::RobotState current_robot_state = lscene->getCurrentState();
  moveit::core::RobotState target_robot_state = current_robot_state;
  double timeout = parameters_.planning_timeout;
  bool success;

  if (!jmg)
  {
    RCLCPP_ERROR(LOGGER, "Could not joint group from %s", group.c_str());
    return false;
  }
  if (goal.empty())
  {
    RCLCPP_ERROR(LOGGER, "Goal is empty");
    return false;
  }

  if (getJointStateGoal(goal, jmg, target_robot_state))
  {
    RCLCPP_INFO(LOGGER, "get a joint state goal");
    success = plan(lscene, target_robot_state, jmg, timeout, robot_trajectory, path_constraints);
  }
  else
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
    auto current_robot_state = lscene->getCurrentState();

    Eigen::Isometry3d target;

    // tip link
    const moveit::core::LinkModel* link;
    // the translation vector from link to global(planning) frame
    Eigen::Isometry3d ik_pose_world;
    if (!utils::getRobotTipForFrame(lscene, jmg, link, ik_pose_world, ik_frame_id))
      return false;

    if (!getPoseGoal(goal, lscene, target) && !getPointGoal(goal, ik_pose_world, lscene, target))
    {
      RCLCPP_ERROR(LOGGER, "Invalid goal type: %s", goal.type().name());
      return false;
    }

    // offset from link to ik_frame
    Eigen::Isometry3d offset =
        lscene->getCurrentState().getGlobalLinkTransform(link).inverse() * ik_pose_world;

    if (!compute_cartesian_path)
    {
      geometry_msgs::msg::Pose end_pose =
          tf2::toMsg(current_robot_state.getGlobalLinkTransform(link));
      RCLCPP_INFO(LOGGER, "Current pose of %s: %s", link->getName().c_str(),
                  geometry_msgs::msg::to_yaml(end_pose).c_str());
      success =
          plan(lscene, *link, offset, target, jmg, timeout, robot_trajectory, path_constraints);
    }
    else
    {
      moveit_msgs::msg::RobotTrajectory robot_traj_msg;
      success =
          planCartesianToPose(group, current_robot_state, *link, offset, target, robot_trajectory);
    }
  }

  time_parametrization_.computeTimeStamps(*robot_trajectory,
                                          parameters_.max_velocity_scaling_factor,
                                          parameters_.max_acceleration_scaling_factor);

  if (success)
  {
    RCLCPP_INFO(LOGGER, "Planning successed");
    return true;
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Fail planning");
    return false;
  }
}

}  // namespace robot_skills
