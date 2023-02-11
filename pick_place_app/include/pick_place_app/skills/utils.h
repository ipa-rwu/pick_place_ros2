#ifndef ROBOT_SKILLS__UTILS_H_
#define ROBOT_SKILLS__UTILS_H_

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <boost/any.hpp>
#include <yaml-cpp/yaml.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "rclcpp/parameter_client.hpp"

namespace robot_skills
{
namespace utils
{
const std::string CURRENT_PKG = "pick_place_app";

bool getRobotTipForFrame(const planning_scene::PlanningSceneConstPtr& scene,
                         const moveit::core::JointModelGroup* jmg,
                         const moveit::core::LinkModel*& robot_link,
                         Eigen::Isometry3d& tip_in_global_frame,
                         const std::string ik_frame_id = "");

template <typename T>
T getValueFromYaml(const YAML::Node& node, const std::string& key);

bool loadPathConstraintsFromYaml(const std::string path_constraints_yaml,
                                 moveit_msgs::msg::Constraints& path_constraints);

bool loadPointsFromYaml(const std::string path_constraints_yaml,
                        geometry_msgs::msg::PoseArray& pose_array);

bool getParameterFromOtherNode(rclcpp::Node::SharedPtr node, const std::string node_name,
                               std::vector<std::string> param_names);
}  // namespace utils

}  // namespace robot_skills

#endif
