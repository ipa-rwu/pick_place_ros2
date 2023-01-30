#include "pick_place_app/skills/compute_path_skill.h"
#include "pick_place_app/skills/utils.h"

#include <moveit/kinematic_constraints/utils.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace robot_skills
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ComputePathSkill");

void ComputePathSkill::Parameters::loadParameters(const rclcpp::Node::SharedPtr& node)
{
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(node, "ik_frame",
                                     ik_frame);  //
  rosparam_shortcuts::shutdownIfError(errors);

  // Optional parameters (default value exists => no shutdown required if
  // loading fails)
  size_t warnings = 0;
  warnings += !rosparam_shortcuts::get(node, "planning_plugin",
                                       planning_plugin);  // e.g. ompl_interface/OMPLPlanner
  warnings += !rosparam_shortcuts::get(node, "planning_timeout",
                                       planning_timeout);  // e.g. 1000s
  warnings += !rosparam_shortcuts::get(node, "num_planning_attempts",
                                       num_planning_attempts);  // e.g. 3

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

  planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(
      robot_model_, node_, "", "planning_plugin", "request_adapters"));
}

ComputePathSkill::~ComputePathSkill() = default;

bool ComputePathSkill::getJointStateGoal(const boost::any& goal,
                                         const moveit::core::JointModelGroup* jmg,
                                         moveit::core::RobotState& state)
{
  try
  {
    // try named joint pose
    const std::string& named_joint_pose = boost::any_cast<std::string>(goal);
    if (!state.setToDefaultValues(jmg, named_joint_pose))
      RCLCPP_ERROR(LOGGER, "Unknown joint pose: %s", named_joint_pose.c_str());
    RCLCPP_INFO(LOGGER, "Get named_joint_pose: %s", named_joint_pose.c_str());
    state.update();
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

    moveit::core::robotStateMsgToRobotState(msg, state, false);
    return true;
  }
  catch (const boost::bad_any_cast&)
  {
  }

  try
  {
    const std::map<std::string, double>& joint_map =
        boost::any_cast<std::map<std::string, double>>(goal);
    const auto& accepted = jmg->getJointModelNames();
    for (const auto& joint : joint_map)
    {
      if (std::find(accepted.begin(), accepted.end(), joint.first) == accepted.end())
        RCLCPP_ERROR(LOGGER, "Joint '%s' is not part of group '%s'", joint.first.c_str(),
                     jmg->getName().c_str());
      state.setVariablePosition(joint.first, joint.second);
    }
    state.update();
    return true;
  }
  catch (const boost::bad_any_cast&)
  {
  }
  return false;
}

bool ComputePathSkill::getPoseGoal(const boost::any& goal,
                                   const planning_scene::PlanningScenePtr& scene,
                                   Eigen::Isometry3d& target)
{
  try
  {
    const geometry_msgs::msg::PoseStamped& msg =
        boost::any_cast<geometry_msgs::msg::PoseStamped>(goal);
    tf2::fromMsg(msg.pose, target);

    // transform target into global frame
    target = scene->getFrameTransform(msg.header.frame_id) * target;
  }
  catch (const boost::bad_any_cast&)
  {
    return false;
  }
  return true;
}

bool ComputePathSkill::getPointGoal(const boost::any& goal, const Eigen::Isometry3d& ik_pose,
                                    const planning_scene::PlanningScenePtr& scene,
                                    Eigen::Isometry3d& target_eigen)
{
  try
  {
    const geometry_msgs::msg::PointStamped& target =
        boost::any_cast<geometry_msgs::msg::PointStamped>(goal);
    Eigen::Vector3d target_point;
    tf2::fromMsg(target.point, target_point);
    // transform target into global frame
    target_point = scene->getFrameTransform(target.header.frame_id) * target_point;

    // retain link orientation
    target_eigen = ik_pose;
    target_eigen.translation() = target_point;
  }
  catch (const boost::bad_any_cast&)
  {
    return false;
  }
  return true;
}

void ComputePathSkill::initMotionPlanRequest(moveit_msgs::msg::MotionPlanRequest& req,
                                             const moveit::core::JointModelGroup* jmg,
                                             double timeout)
{
  req.group_name = jmg->getName();
  req.planner_id = parameters_.planning_plugin;
  req.allowed_planning_time = timeout;
  req.start_state.is_diff = true;  // we don't specify an extra start state

  req.num_planning_attempts = parameters_.num_planning_attempts;
}

bool ComputePathSkill::plan(const moveit::core::LinkModel& link, const Eigen::Isometry3d& offset,
                            const Eigen::Isometry3d& target_eigen,
                            const moveit::core::JointModelGroup* jmg, double timeout,
                            robot_trajectory::RobotTrajectoryPtr& result,
                            const moveit_msgs::msg::Constraints& path_constraints)
{
  moveit_msgs::msg::MotionPlanRequest req;
  initMotionPlanRequest(req, jmg, timeout);

  planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = lscene->getPlanningFrame();
  target.pose = tf2::toMsg(target_eigen * offset.inverse());

  req.goal_constraints.resize(1);
  req.goal_constraints[0] =
      kinematic_constraints::constructGoalConstraints(link.getName(), target,
                                                      parameters_.goal_position_tolerance,
                                                      parameters_.goal_orientation_tolerance);
  req.path_constraints = path_constraints;

  planning_interface::MotionPlanResponse res;
  bool success = planning_pipeline_->generatePlan(lscene, req, res);
  result = res.trajectory_;
  time_parametrization_.computeTimeStamps(*result, parameters_.max_velocity_scaling_factor,
                                          parameters_.max_acceleration_scaling_factor);
  return success;
}

bool ComputePathSkill::plan(const planning_scene::PlanningSceneConstPtr& target_scene,
                            const moveit::core::JointModelGroup* jmg, double timeout,
                            robot_trajectory::RobotTrajectoryPtr& result,
                            const moveit_msgs::msg::Constraints& path_constraints)
{
  moveit_msgs::msg::MotionPlanRequest req;
  initMotionPlanRequest(req, jmg, timeout);

  planning_interface::MotionPlanResponse res;
  planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
  req.goal_constraints.resize(1);
  req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
      target_scene->getCurrentState(), jmg, parameters_.goal_joint_tolerance);
  req.path_constraints = path_constraints;
  /* Now, call the pipeline and check whether planning was successful. */
  bool success = planning_pipeline_->generatePlan(lscene, req, res);
  result = res.trajectory_;
  time_parametrization_.computeTimeStamps(*result, parameters_.max_velocity_scaling_factor,
                                          parameters_.max_acceleration_scaling_factor);
  return success;
}

bool ComputePathSkill::planCartesian(const std::string& group,
                                     const moveit::core::RobotState& current_robot_state,
                                     const std::string& plan_frame_id,
                                     const moveit::core::LinkModel& link,
                                     const Eigen::Isometry3d& offset,
                                     const Eigen::Isometry3d& target_eigen,
                                     robot_trajectory::RobotTrajectoryPtr& result)
{
  move_group.reset();
  move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group);

  geometry_msgs::msg::Pose end_pose =
      tf2::toMsg(current_robot_state.getGlobalLinkTransform(&link) * offset.inverse());
  RCLCPP_DEBUG(LOGGER, "end pose: %s", geometry_msgs::msg::to_yaml(end_pose).c_str());

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = plan_frame_id;
  target.pose = tf2::toMsg(target_eigen * offset.inverse());
  RCLCPP_DEBUG(LOGGER, "target pose: %s", geometry_msgs::msg::to_yaml(target.pose).c_str());

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(end_pose);
  geometry_msgs::msg::Pose temp_pose = end_pose;
  temp_pose.position.x = target.pose.position.x;
  waypoints.push_back(temp_pose);
  temp_pose.position.y = target.pose.position.y;
  waypoints.push_back(temp_pose);
  temp_pose.position.z = target.pose.position.z;
  waypoints.push_back(temp_pose);

  // plan to Cartesian target

  double max_fraction = 0;
  double current_fraction = 0;
  auto robot_trajectory_msg = moveit_msgs::msg::RobotTrajectory();

  int try_plan = 0;
  while (try_plan < parameters_.num_planning_attempts)
  {
    try_plan++;
    moveit_msgs::msg::RobotTrajectory tmp_robot_trajectory;
    current_fraction = move_group->computeCartesianPath(
        waypoints, parameters_.step_size, parameters_.jump_threshold, tmp_robot_trajectory);
    if (current_fraction > max_fraction)
    {
      max_fraction = current_fraction;
      robot_trajectory_msg = tmp_robot_trajectory;
    }
    RCLCPP_INFO(LOGGER, "(%.2f%% achieved)", current_fraction * 100.00);
  }
  bool success = false;
  if (max_fraction < 0.5)
  {
    return success;
  }
  result->setRobotTrajectoryMsg(current_robot_state, robot_trajectory_msg);

  time_parametrization_.computeTimeStamps(*result, parameters_.max_velocity_scaling_factor,
                                          parameters_.max_acceleration_scaling_factor);

  success = true;
  return success;
}

bool ComputePathSkill::compute(planning_scene::PlanningScenePtr& scene, const std::string& group,
                               const boost::any& goal,
                               robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
                               const moveit_msgs::msg::Constraints& path_constraints,
                               bool compute_cartesian_path)
{
  auto scene_current = scene->diff();
  auto scene_diff = scene->diff();
  const moveit::core::RobotModelConstPtr& robot_model = scene_diff->getRobotModel();

  double timeout = parameters_.planning_timeout;
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group);
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

  bool success = false;

  if (getJointStateGoal(goal, jmg, scene_diff->getCurrentStateNonConst()))
  {
    success = plan(scene_diff, jmg, timeout, robot_trajectory, path_constraints);
  }
  else
  {
    Eigen::Isometry3d target;
    geometry_msgs::msg::PoseStamped ik_pose_msg;

    const moveit::core::LinkModel* link;
    Eigen::Isometry3d ik_pose_world;
    if (!utils::getRobotTipForFrame(parameters_.ik_frame, *scene, jmg, link, ik_pose_world))
      return false;

    if (!getPoseGoal(goal, scene, target) && !getPointGoal(goal, ik_pose_world, scene, target))
    {
      RCLCPP_ERROR(LOGGER, "Invalid goal type: %s", goal.type().name());
      return false;
    }

    // offset from link to ik_frame
    Eigen::Isometry3d offset =
        scene->getCurrentState().getGlobalLinkTransform(link).inverse() * ik_pose_world;

    planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
    auto current_robot_state = lscene->getCurrentState();

    if (!compute_cartesian_path)
    {
      success = plan(*link, offset, target, jmg, timeout, robot_trajectory, path_constraints);
    }
    else
    {
      moveit_msgs::msg::RobotTrajectory robot_traj_msg;
      success = planCartesian(group, lscene->getCurrentState(), lscene->getPlanningFrame(), *link,
                              offset, target, robot_trajectory);
    }
  }

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
