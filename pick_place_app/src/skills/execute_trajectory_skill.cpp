#include "pick_place_app/skills/execute_trajectory_skill.h"

namespace robot_skills
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("execute_trajectory_skill");

ExecuteTrajectorySkill::ExecuteTrajectorySkill(rclcpp::Node::SharedPtr node) : node_(node)
{
  RCLCPP_INFO(LOGGER, "create execute trajectory skill");
}

ExecuteTrajectorySkill::~ExecuteTrajectorySkill() = default;

bool ExecuteTrajectorySkill::execute_trajectory(const std::string group_name,
                                                const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  move_group.reset();
  move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
  moveit::core::MoveItErrorCode res = move_group->execute(trajectory);
  if (res != moveit::core::MoveItErrorCode::SUCCESS)
  {
    return false;
  }
  else
  {
    return true;
  }
}
}  // namespace robot_skills
