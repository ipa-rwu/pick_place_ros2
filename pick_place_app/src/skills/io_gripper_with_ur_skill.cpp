#include "pick_place_app/skills/io_gripper_with_ur_skill.h"
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace robot_skills
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("io_gripper_with_ur_skill");

void IOGripperWithURSkill::Parameters::loadParameters(const rclcpp::Node::SharedPtr& node)
{
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(node, "io_service_name",
                                     io_service_name);  //
  RCLCPP_INFO(LOGGER, "param: %s", io_service_name.c_str());

  rosparam_shortcuts::shutdownIfError(errors);
}

IOGripperWithURSkill::IOGripperWithURSkill(rclcpp::Node::SharedPtr node,
                                           const IOGripperWithURSkill::Parameters& parameters)
  : node_(node), parameters_(parameters)
{
  RCLCPP_INFO(LOGGER, "try to connect to %s", parameters_.io_service_name.c_str());
  client = node_->create_client<ur_msgs::srv::SetIO>(parameters_.io_service_name.c_str());
}

IOGripperWithURSkill::~IOGripperWithURSkill() = default;

/**
 * @brief
 *
 * @param command "open" or "close"
 */
bool IOGripperWithURSkill::setGripperState(const std::string command)
{
  int open = 1;
  if (command == "close")
  {
    open = 0;
  }
  RCLCPP_INFO(LOGGER, "Get command: %s, try to connect to: %s", command.c_str(),
              parameters_.io_service_name.c_str());

  bool success = false;
  if (client->wait_for_service(std::chrono::duration<double>(parameters_.timeout)))
  {
    RCLCPP_INFO(LOGGER, "Connected to server : %s", parameters_.io_service_name.c_str());
    auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
    req->fun = req->FUN_SET_DIGITAL_OUT;
    req->state = 1;
    req->pin = static_cast<int8_t>(open);  // Pin 1 is open

    auto res_future = client->async_send_request(req);
    success = res_future.get()->success;
    RCLCPP_INFO(LOGGER, "%s gripper initialise : %s", ((open == 1) ? "Open" : "Close"),
                ((success == false) ? "Failed" : "Succeeded"));

    req->state = 0;
    res_future = client->async_send_request(req);
    success = res_future.get()->success;
    RCLCPP_INFO(LOGGER, "%s gripper conclude : %s", ((open == 1) ? "Open" : "Close"),
                ((success == false) ? "Failed" : "Succeeded"));
  }
  return success;
}
}  // namespace robot_skills
