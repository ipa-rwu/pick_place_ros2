#include "pick_place_app/skills/execute_trajectory_skill.h"

namespace robot_skills
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("execute_trajectory_skill");

ExecuteTrajectorySkill::ExecuteTrajectorySkill(rclcpp::Node::SharedPtr node,
                                               moveit_cpp::MoveItCppPtr& moveit_cpp_ptr)
  : node_(node), moveit_cpp_ptr_(moveit_cpp_ptr)
{
  RCLCPP_INFO(LOGGER, "create execute trajectory skill");
}

ExecuteTrajectorySkill::~ExecuteTrajectorySkill() = default;

bool ExecuteTrajectorySkill::execute_trajectory(
    const std::string group_name, const robot_trajectory::RobotTrajectoryPtr& robot_trajectory)
{
  bool success = moveit_cpp_ptr_->execute(group_name, robot_trajectory);
  return success;
}
}  // namespace robot_skills
