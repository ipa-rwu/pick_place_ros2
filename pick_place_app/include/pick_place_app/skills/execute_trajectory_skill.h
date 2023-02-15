#ifndef ROBOT_SKILLS__EXECUTE_TRAJECTORY_SKILL_H_
#define ROBOT_SKILLS__EXECUTE_TRAJECTORY_SKILL_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/moveit_cpp/moveit_cpp.h>

#include <boost/any.hpp>

#include <memory>

namespace robot_skills
{
class ExecuteTrajectorySkill
{
public:
  /**
   * @brief
   *
   */
  RCLCPP_SMART_PTR_DEFINITIONS(ExecuteTrajectorySkill)

  ExecuteTrajectorySkill(rclcpp::Node::SharedPtr node, moveit_cpp::MoveItCppPtr& moveit_cpp_ptr);

  ~ExecuteTrajectorySkill();

  bool execute_trajectory(const std::string group_name,
                          const robot_trajectory::RobotTrajectoryPtr& robot_trajectory);

private:
  rclcpp::Node::SharedPtr node_;
  moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
};

}  // namespace robot_skills

#endif
