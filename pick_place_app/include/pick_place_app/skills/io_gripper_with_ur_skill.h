#ifndef ROBOT_SKILLS__IO_GRIPPER_WITH_UR_SKILL_H_
#define ROBOT_SKILLS__IO_GRIPPER_WITH_UR_SKILL_H_

#include <rclcpp/rclcpp.hpp>
#include "ur_msgs/srv/set_io.hpp"

#include <memory>

namespace robot_skills
{
class IOGripperWithURSkill
{
public:
  /**
   * @brief
   *
   */
  RCLCPP_SMART_PTR_DEFINITIONS(IOGripperWithURSkill)

  struct Parameters
  {
    std::string io_service_name;
    double timeout;

    void loadParameters(const rclcpp::Node::SharedPtr& node)
    {
      std::string ns = "io_gripper_with_ur_skill.";
      node->get_parameter_or(ns + "io_service_name", io_service_name,
                             std::string("mock_io_server/set_io"));
      node->get_parameter_or(ns + "timeout", timeout, 5.0);
    }
  };

  IOGripperWithURSkill(rclcpp::Node::SharedPtr node,
                       const IOGripperWithURSkill::Parameters& parameters);

  ~IOGripperWithURSkill();

  bool setGripperState(const std::string command);

protected:
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client;

private:
  rclcpp::Node::SharedPtr node_;
  IOGripperWithURSkill::Parameters parameters_;
};

}  // namespace robot_skills

#endif
