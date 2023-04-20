#include "pick_place_app/skills/compute_path_with_moveitcpp_skill.h"
#include "pick_place_app/skills/utils.h"

#include <moveit/kinematic_constraints/utils.h>
#include <geometry_msgs/msg/vector3.hpp>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace robot_skills
{
using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("compute_path_moveitcpp_skill");

ComputePathWithMoveItCppSkill::ComputePathWithMoveItCppSkill(
    rclcpp::Node::SharedPtr node, const ComputePathWithMoveItCppSkill::Parameters& parameters,
    moveit_cpp::MoveItCppPtr& moveit_cpp_ptr)
  : node_(node), parameters_(parameters), moveit_cpp_ptr_(moveit_cpp_ptr)

{
  psm_ = moveit_cpp_ptr_->getPlanningSceneMonitor();
  psm_->providePlanningSceneService(parameters_.GET_PLANNING_SCENE_SERVICE_NAME);
  set_planner_called_ = false;
  RCLCPP_INFO(LOGGER, "create execute compute_path_moveitcpp_skill skill");
}

ComputePathWithMoveItCppSkill::~ComputePathWithMoveItCppSkill() = default;

void ComputePathWithMoveItCppSkill::initPlanComponentParameters(
    moveit_cpp::PlanningComponent::PlanRequestParameters& plan_params)
{
  set_planner_called_ = false;
  plan_params = moveit_cpp::PlanningComponent::PlanRequestParameters();
  if (parameters_.planner_id != "")
    plan_params.planner_id = parameters_.planner_id;
  if (parameters_.planning_pipeline != "")
    plan_params.planning_pipeline = parameters_.planning_pipeline;
  plan_params.planning_attempts = parameters_.planning_attempts;
  plan_params.planning_time = parameters_.planning_time;
  plan_params.max_velocity_scaling_factor = parameters_.max_velocity_scaling_factor;
  plan_params.max_acceleration_scaling_factor = parameters_.max_acceleration_scaling_factor;
}

void ComputePathWithMoveItCppSkill::setPlanner(const std::string& planning_pipeline,
                                               const std::string& planner_id)
{
  initPlanComponentParameters(plan_params_);
  if (!planning_pipeline.empty())
  {
    plan_params_.planning_pipeline = planning_pipeline;
  }
  if (!planner_id.empty())
  {
    plan_params_.planner_id = planner_id;
  }
  set_planner_called_ = true;
  RCLCPP_INFO(LOGGER, "setPlanner: Using planning_pipeline: %s ",
              plan_params_.planning_pipeline.c_str());
}

bool ComputePathWithMoveItCppSkill::getJointStateGoal(const boost::any& goal,
                                                      const moveit::core::JointModelGroup* jmg,
                                                      moveit::core::RobotState& target_state)
{
  try
  {
    // try named joint pose
    const std::string& named_joint_pose = boost::any_cast<std::string>(goal);
    if (!target_state.setToDefaultValues(jmg, named_joint_pose))
      RCLCPP_ERROR(LOGGER, "Unknown joint pose: %s", named_joint_pose.c_str());
    RCLCPP_INFO(LOGGER, "Get named_joint_pose: %s", named_joint_pose.c_str());
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

bool ComputePathWithMoveItCppSkill::getPoseGoal(const boost::any& goal,
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

bool ComputePathWithMoveItCppSkill::getPointGoal(const boost::any& goal,
                                                 const Eigen::Isometry3d& ik_pose,
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

bool ComputePathWithMoveItCppSkill::plan(const moveit::core::RobotState& target_robot_state,
                                         const moveit::core::JointModelGroup* jmg,
                                         robot_trajectory::RobotTrajectoryPtr& result,
                                         const moveit_msgs::msg::Constraints& path_constraints)
{
  moveit_cpp::PlanningComponent::PlanRequestParameters plan_params;
  if (set_planner_called_)
  {
    plan_params = plan_params_;
    set_planner_called_ = false;
  }
  else
  {
    initPlanComponentParameters(plan_params);
  }
  bool success = false;

  // Create planning component
  auto planning_components =
      std::make_shared<moveit_cpp::PlanningComponent>(jmg->getName(), moveit_cpp_ptr_);

  auto start_robot_state = moveit_msgs::msg::RobotState();
  moveit::core::robotStateToRobotStateMsg(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState(), start_robot_state);

  auto target_robot_state_msg = moveit_msgs::msg::RobotState();
  moveit::core::robotStateToRobotStateMsg(target_robot_state, target_robot_state_msg);

  // set Path Constraints
  planning_components->setPathConstraints(path_constraints);

  std::vector<moveit_msgs::msg::Constraints> goal_constraints;

  goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
      target_robot_state, jmg, parameters_.goal_joint_tolerance));

  // Copy goal constraint into planning component
  planning_components->setStartStateToCurrentState();
  planning_components->setGoal(goal_constraints);

  // plan
  auto plan_solution = planning_components->plan(plan_params);

  if (plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Planning fail");
    success = false;
    return success;
  }

  success = true;
  RCLCPP_INFO(LOGGER, "Planning success");

  result = plan_solution.trajectory;
  time_parametrization_.computeTimeStamps(*result, parameters_.max_velocity_scaling_factor,
                                          parameters_.max_acceleration_scaling_factor);
  return success;
}

bool ComputePathWithMoveItCppSkill::plan(const moveit::core::LinkModel& link,
                                         const Eigen::Isometry3d& offset,
                                         const Eigen::Isometry3d& target_eigen,
                                         const moveit::core::JointModelGroup* jmg,
                                         robot_trajectory::RobotTrajectoryPtr& result,
                                         const moveit_msgs::msg::Constraints& path_constraints)
{
  moveit_cpp::PlanningComponent::PlanRequestParameters plan_params;
  bool success = false;
  if (set_planner_called_)
  {
    plan_params = plan_params_;
    set_planner_called_ = false;
  }
  else
  {
    initPlanComponentParameters(plan_params);
  }

  auto start_robot_state = moveit_msgs::msg::RobotState();
  moveit::core::robotStateToRobotStateMsg(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState(), start_robot_state);
  RCLCPP_INFO(LOGGER, "Ready for planning: robot start state: %s",
              moveit_msgs::msg::to_yaml(start_robot_state).c_str());

  // Create planning component
  auto planning_components =
      std::make_shared<moveit_cpp::PlanningComponent>(jmg->getName(), moveit_cpp_ptr_);

  // set Path Constraints
  planning_components->setPathConstraints(path_constraints);

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = planning_scene_monitor::LockedPlanningSceneRO(psm_)->getPlanningFrame();
  target.pose = tf2::toMsg(target_eigen * offset.inverse());
  RCLCPP_INFO(LOGGER, "Prepare for planning, goal:\n target frame: %s\n posestamp: %s",
              link.getName().c_str(), geometry_msgs::msg::to_yaml(target).c_str());

  std::vector<moveit_msgs::msg::Constraints> goal_constraints;
  goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
      link.getName(), target, parameters_.goal_position_tolerance,
      parameters_.goal_orientation_tolerance));

  // Copy goal constraint into planning component
  planning_components->setGoal(goal_constraints);
  planning_components->setStartStateToCurrentState();
  // plan
  auto plan_solution = planning_components->plan(plan_params);

  if (plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    success = false;
    return success;
  }

  success = true;
  result = plan_solution.trajectory;
  time_parametrization_.computeTimeStamps(*result, parameters_.max_velocity_scaling_factor,
                                          parameters_.max_acceleration_scaling_factor);
  return success;
}

bool ComputePathWithMoveItCppSkill::planCartesianMoveGroup(
    const std::string& group, const std::vector<geometry_msgs::msg::Pose>& waypoints,
    robot_trajectory::RobotTrajectoryPtr& result)
{
  move_group_.reset();
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group);

  double max_fraction = 0;
  double current_fraction = 0;
  auto robot_trajectory_msg = moveit_msgs::msg::RobotTrajectory();

  int try_plan = 0;
  bool success;

  while (try_plan < parameters_.planning_attempts)
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
  result->setRobotTrajectoryMsg(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState(), robot_trajectory_msg);

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

bool ComputePathWithMoveItCppSkill::computeRelative(
    const std::string& group, geometry_msgs::msg::Vector3 direction,
    std::vector<robot_trajectory::RobotTrajectoryPtr>& robot_trajectories,
    const std::string& ik_frame_id)
{
  const auto robot_model = planning_scene_monitor::LockedPlanningSceneRO(psm_)->getRobotModel();
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group);

  const planning_scene::PlanningScenePtr lscene = [&] {
    planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
    return planning_scene::PlanningScene::clone(ls);
  }();
  // transfer robot state to a pose of link in global(planning) frame
  // tip link
  const moveit::core::LinkModel* link;
  // the translation vector from link to global(planning) frame
  Eigen::Isometry3d ik_pose_world;
  if (!utils::getRobotTipForFrame(lscene, jmg, link, ik_pose_world, ik_frame_id))
    return false;

  // offset from link to ik_frame
  Eigen::Isometry3d offset =
      lscene->getCurrentState().getGlobalLinkTransform(link).inverse() * ik_pose_world;

  geometry_msgs::msg::Pose end_pose =
      tf2::toMsg((planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState())
                     .getGlobalLinkTransform(link));
  RCLCPP_INFO(LOGGER, "computeRelative: Current pose of %s: %s", link->getName().c_str(),
              geometry_msgs::msg::to_yaml(end_pose).c_str());

  bool success;

  std::vector<geometry_msgs::msg::Pose> waypoints;

  if (abs(direction.x) > 0)
  {
    end_pose.position.x = end_pose.position.x + direction.x;
    // waypoints.push_back(end_pose);
  }
  if (abs(direction.y) > 0)
  {
    end_pose.position.y = end_pose.position.y + direction.y;
    // waypoints.push_back(end_pose);
  }
  if (abs(direction.z) > 0)
  {
    end_pose.position.z = end_pose.position.z + direction.z;
    waypoints.push_back(end_pose);
  }

  const auto available_pipelines = moveit_cpp_ptr_->getPlanningPipelineNames(group);

  if (available_pipelines.find(parameters_.PILZ_PLANNING_PIPELINE) != available_pipelines.end())
  {
    for (const auto& waypoint : waypoints)
    {
      geometry_msgs::msg::PoseStamped tmp_pose;
      tmp_pose.header.frame_id =
          planning_scene_monitor::LockedPlanningSceneRO(psm_)->getPlanningFrame();
      tmp_pose.pose = waypoint;
      RCLCPP_INFO(LOGGER, "plan from relative position, pass waypoins: %s",
                  geometry_msgs::msg::to_yaml(tmp_pose).c_str());
      robot_trajectory::RobotTrajectoryPtr traj;
      Eigen::Isometry3d target;
      if (getPoseGoal(tmp_pose, lscene, target))
      {
        setPlanner("pilz_industrial_motion_planner", "LIN");
        success = plan(*link, offset, target, jmg, traj);

        if (!success)
        {
          RCLCPP_ERROR(LOGGER, "computeRelative, FAIL!!!");
          return success;
        }
        else
        {
          RCLCPP_INFO(LOGGER, "computeRelative, success!!!");
          time_parametrization_.computeTimeStamps(*traj, parameters_.max_velocity_scaling_factor,
                                                  parameters_.max_acceleration_scaling_factor);
          robot_trajectories.push_back(traj);
        }
      }
      else
        return false;
    }
  }
  else
  {
    // plan to Cartesian target
    robot_trajectory::RobotTrajectoryPtr traj;
    success = planCartesianMoveGroup(group, waypoints, traj);
    time_parametrization_.computeTimeStamps(*traj, parameters_.max_velocity_scaling_factor,
                                            parameters_.max_acceleration_scaling_factor);

    robot_trajectories.push_back(traj);
  }

  return success;
}

bool ComputePathWithMoveItCppSkill::checkCollision(
    const planning_scene::PlanningSceneConstPtr& current_scene)
{
  moveit_msgs::msg::RobotState current_robot_state_msg;
  moveit::core::robotStateToRobotStateMsg(current_scene->getCurrentState(), current_robot_state_msg);
  RCLCPP_INFO(LOGGER, "Before planning, current robot state is: %s",
              moveit_msgs::msg::to_yaml(current_robot_state_msg).c_str());
  collision_detection::CollisionRequest collision_req;
  collision_detection::CollisionResult collision_res;

  current_scene->checkCollision(collision_req, collision_res);
  for (collision_detection::CollisionResult::ContactMap::const_iterator contacts_iter =
           collision_res.contacts.begin();
       contacts_iter != collision_res.contacts.end(); ++contacts_iter)
  {
    RCLCPP_INFO(LOGGER, "Link [%s]: has collision with Link[%s] ",
                contacts_iter->first.first.c_str(), contacts_iter->first.second.c_str());
  }
  RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (collision_res.collision ? "in" : "not in")
                                                 << " self collision");
  return collision_res.collision;
}

bool ComputePathWithMoveItCppSkill::planCartesianToPose(
    const std::string& group, const moveit::core::RobotState& current_robot_state,
    const moveit::core::LinkModel& link, const Eigen::Isometry3d& offset,
    const Eigen::Isometry3d& target_eigen, robot_trajectory::RobotTrajectoryPtr& result)
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
  success = planCartesianMoveGroup(group, waypoints, result);
  return success;
}

bool ComputePathWithMoveItCppSkill::computePath(
    const std::string& group, const boost::any& goal,
    robot_trajectory::RobotTrajectoryPtr& robot_trajectory, const std::string& ik_frame_id,
    bool compute_cartesian_path, const moveit_msgs::msg::Constraints& path_constraints)
{
  const moveit::core::JointModelGroup* jmg =
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getRobotModel()->getJointModelGroup(
          group);
  moveit::core::RobotState current_robot_state =
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState();
  moveit::core::RobotState target_robot_state = current_robot_state;

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

  moveit_msgs::msg::RobotState current_state_msg;
  moveit::core::robotStateToRobotStateMsg(current_robot_state, current_state_msg);
  RCLCPP_DEBUG(LOGGER, "Before transfering goal, current robot state: %s",
               moveit_msgs::msg::to_yaml(current_state_msg).c_str());

  if (getJointStateGoal(goal, jmg, target_robot_state))
  {
    success = plan(target_robot_state, jmg, robot_trajectory, path_constraints);
    return success;
  }
  else
  {
    const planning_scene::PlanningScenePtr lscene = [&] {
      planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
      return planning_scene::PlanningScene::clone(ls);
    }();

    current_robot_state = lscene->getCurrentState();

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

      checkCollision(lscene);
      success = plan(*link, offset, target, jmg, robot_trajectory, path_constraints);
    }
    else
    {
      checkCollision(lscene);
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
