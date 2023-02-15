#include "pick_place_app/app/point_to_point.h"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("point_to_point_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("point_to_point_demo", "", options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    executor.remove_node(node->get_node_base_interface());
  });
  rclcpp::sleep_for(std::chrono::seconds(1));

  robot_application::PointToPoinTask::Parameters point_to_point_parameters;
  point_to_point_parameters.loadParameters(node);

  robot_application::PointToPoinTask point_to_point_task(node, point_to_point_parameters);

  while (rclcpp::ok())
  {
    point_to_point_task.executeTask();
  }

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
