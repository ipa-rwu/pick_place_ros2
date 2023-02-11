#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/robot_state.h>
#include <moveit/robot_state/conversions.h>

using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_psm");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("test_psm_demo", "", options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    executor.remove_node(node->get_node_base_interface());
  });

  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(node, "robot_description"));

  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();

  auto robot_state_msg = moveit_msgs::msg::RobotState();
  RCLCPP_INFO(LOGGER,
              "StateUpdateFrequency: %f, "
              "PlanningScenePublishingFrequency: %f ",
              psm->getStateUpdateFrequency(), psm->getPlanningScenePublishingFrequency());

  while (rclcpp::ok())
  {
    auto last_time = psm->getLastUpdateTime();
    RCLCPP_INFO(LOGGER, "last_time: %f, now: %f", last_time.seconds(), node->now().seconds());

    robot_state_msg = moveit_msgs::msg::RobotState();
    moveit::core::robotStateToRobotStateMsg(
        planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState(), robot_state_msg);
    RCLCPP_INFO(LOGGER, "Current state from lscene: %s",
                moveit_msgs::msg::to_yaml(robot_state_msg).c_str());

    rclcpp::sleep_for(2s);
  }

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
