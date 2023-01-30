#ifndef ROBOT_SKILLS__EXECUTE_TRAJECTORY_SKILL_H_
#define ROBOT_SKILLS__EXECUTE_TRAJECTORY_SKILL_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
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

  ExecuteTrajectorySkill(rclcpp::Node::SharedPtr node);

  ~ExecuteTrajectorySkill();

  bool execute_trajectory(const std::string group_name,
                          const moveit_msgs::msg::RobotTrajectory& trajectory);

protected:
  moveit::planning_interface::MoveGroupInterfacePtr move_group;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace robot_skills

#endif
