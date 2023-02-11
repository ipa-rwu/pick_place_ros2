#include "pick_place_app/skills/utils.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace robot_skills
{
namespace utils
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("RobotSkills_utils");

bool getRobotTipForFrame(const planning_scene::PlanningSceneConstPtr& scene,
                         const moveit::core::JointModelGroup* jmg,
                         const moveit::core::LinkModel*& robot_link,
                         Eigen::Isometry3d& tip_in_global_frame, const std::string ik_frame_id)
{
  auto get_tip = [&jmg]() -> const moveit::core::LinkModel* {
    // determine IK frame from group
    std::vector<const moveit::core::LinkModel*> tips;
    jmg->getEndEffectorTips(tips);
    if (tips.size() != 1)
    {
      return nullptr;
    }
    RCLCPP_DEBUG(LOGGER, "getRobotTipForFrame: get tip: %s", tips[0]->getName().c_str());
    return tips[0];
  };

  if (ik_frame_id.empty())
  {
    robot_link = get_tip();
    if (!robot_link)
    {
      RCLCPP_ERROR(LOGGER, "missing ik_frame");
      return false;
    }
    // transfer of tip (tip pose) in planning frame
    tip_in_global_frame = scene->getCurrentState().getGlobalLinkTransform(robot_link);
  }
  else
  {
    robot_link = nullptr;
    bool found = false;
    // if found, get transfer ik_frame_id (ik_frame pose) in planning frame
    auto ref_frame = scene->getCurrentState().getFrameInfo(ik_frame_id, robot_link, found);
    if (!found && !ik_frame_id.empty())
    {
      std::stringstream ss;
      ss << "ik_frame specified in unknown frame '" << ik_frame_id << "'";
      RCLCPP_ERROR(LOGGER, "%s", ss.str().c_str());
      return false;
    }
    if (!robot_link)
      robot_link = get_tip();
    if (!robot_link)
    {
      RCLCPP_ERROR(LOGGER, "ik_frame doesn't specify a link frame");

      return false;
    }

    if (found)
    {  // use robot link's frame as reference by default
      tip_in_global_frame = ref_frame;
    }
    else
    {
      tip_in_global_frame = scene->getCurrentState().getGlobalLinkTransform(robot_link);
    }
  }

  return true;
}

template <typename T>
T getValueFromYaml(const YAML::Node& node, const std::string& key)
{
  try
  {
    return node[key].as<T>();
  }
  catch (YAML::Exception& e)
  {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}

bool loadPathConstraintsFromYaml(const std::string path_constraints_yaml,
                                 moveit_msgs::msg::Constraints& path_constraints)
{
  std::string params_file = ament_index_cpp::get_package_share_directory(CURRENT_PKG) + "/config/" +
                            path_constraints_yaml;
  YAML::Node config = YAML::LoadFile(params_file);
  auto doc = config["path_constraints"];
  try
  {
    auto path_constraint_name = utils::getValueFromYaml<std::string>(doc, "name");
    if (path_constraint_name.empty())
    {
      std::stringstream ss;
      ss << "The name tag was empty in '" << path_constraints_yaml << "' file ";
      RCLCPP_ERROR(LOGGER, "%s", ss.str().c_str());
      return false;
    }
    path_constraints.name = path_constraint_name;
  }
  catch (YAML::Exception& e)
  {
    std::stringstream ss;
    ss << "The name tag was undefined in '" << path_constraints_yaml << "' file ";
    RCLCPP_ERROR(LOGGER, "%s", ss.str().c_str());
    return false;
  }

  try
  {
    for (const auto joint_doc : doc["joint_constraints"])
    {
      moveit_msgs::msg::JointConstraint joint_constraint;
      joint_constraint.joint_name = utils::getValueFromYaml<std::string>(joint_doc, "joint_name");
      joint_constraint.tolerance_above =
          utils::getValueFromYaml<double>(joint_doc, "tolerance_above");
      joint_constraint.tolerance_below =
          utils::getValueFromYaml<double>(joint_doc, "tolerance_below");
      joint_constraint.weight = utils::getValueFromYaml<double>(joint_doc, "weight");

      path_constraints.joint_constraints.push_back(joint_constraint);
    }
  }
  catch (YAML::Exception& e)
  {
    RCLCPP_WARN(LOGGER, "joint_constraints aren't defined in %s", path_constraints_yaml.c_str());
  }
  return true;
  // RWU TODO: parser PositionConstraint OrientationConstraint  VisibilityConstraint
}

bool loadPointsFromYaml(const std::string waypoints_file, geometry_msgs::msg::PoseArray& pose_array)
{
  std::string params_file =
      ament_index_cpp::get_package_share_directory(CURRENT_PKG) + "/config/" + waypoints_file;
  YAML::Node config = YAML::LoadFile(params_file);
  auto doc = config["waypoints"];
  auto doc_poses = doc["poses"];
  try
  {
    auto frame_id = utils::getValueFromYaml<std::string>(doc, "frame_id");
    if (frame_id.empty())
    {
      std::stringstream ss;
      ss << "The frame_id tag was empty in '" << waypoints_file << "' file ";
      RCLCPP_ERROR(LOGGER, "%s", ss.str().c_str());
      return false;
    }
    pose_array.header.frame_id = frame_id;
  }
  catch (YAML::Exception& e)
  {
    RCLCPP_ERROR(LOGGER, "frame_id aren't defined in %s", waypoints_file.c_str());
    return false;
  }

  try
  {
    for (const auto doc_pose : doc_poses)
    {
      geometry_msgs::msg::Pose tmp;
      tmp.position.x = utils::getValueFromYaml<double>(doc_pose["position"], "x");
      tmp.position.y = utils::getValueFromYaml<double>(doc_pose["position"], "y");
      tmp.position.z = utils::getValueFromYaml<double>(doc_pose["position"], "z");

      pose_array.poses.push_back(tmp);
    }
  }
  catch (YAML::Exception& e)
  {
    RCLCPP_WARN(LOGGER, "pose x,y,z aren't defined in %s", waypoints_file.c_str());
    return false;
  }
  return true;
}

bool getParameterFromOtherNode(rclcpp::Node::SharedPtr node, const std::string node_name,
                               std::vector<std::string> param_names)
{
  auto asynchronous_client = std::make_shared<rclcpp::AsyncParametersClient>(node, node_name);
  asynchronous_client->wait_for_service();

  auto callback = [node](std::shared_future<std::vector<rclcpp::Parameter>> future) {
    std::stringstream ss;
    auto results = future.get();
    for (auto& parameter : results)
    {
      RCLCPP_INFO(LOGGER, "Get parameter from /move_group: %s", parameter.get_name().c_str());
      if (node->has_parameter(parameter.get_name()))
      {
        RCLCPP_INFO(LOGGER,
                    "remapParameters: node: [%s] have parameter: "
                    "[%s], will set this parameter",
                    node->get_name(), parameter.get_name().c_str());
        node->set_parameter(rclcpp::Parameter(parameter.get_name(), parameter.value_to_string()));
      }
      else
      {
        RCLCPP_INFO(LOGGER,
                    "remapParameters: node: [%s] don't have parameter: [%s], "
                    "will declear value",
                    node->get_name(), parameter.get_name().c_str());
        node->declare_parameter<std::string>(parameter.get_name(), parameter.value_to_string());
      }
    }
  };

  auto parameters_future = asynchronous_client->get_parameters(param_names, callback);

  for (const auto& param_name : param_names)
  {
    auto start_time = std::chrono::steady_clock::now();
    while (!node->has_parameter(param_name) && rclcpp::ok() &&
           (std::chrono::steady_clock::now() - start_time) < std::chrono::seconds(5))
    {
      RCLCPP_WARN_ONCE(LOGGER, "didn't have Parameter [%s] yet", param_name.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  for (const auto& param_name : param_names)
  {
    if (!node->has_parameter(param_name))
      return false;
  }
  return true;
}

}  // namespace utils
}  // namespace robot_skills
