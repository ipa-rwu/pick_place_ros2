#include "pick_place_app/app/pick_place_static_task.h"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_static_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pick_place_static_demo", "", options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    executor.remove_node(node->get_node_base_interface());
  });

  robot_application::PickPlaceStaticTask::Parameters parameters;
  parameters.loadParameters(node);

  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(node, "robot_description"));

  // robot_application::PickPlaceStaticTask pick_place_static(node, parameters);

  // pick_place_static.executeTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
