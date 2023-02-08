#include "pick_place_app/skills/utils.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace robot_skills
{
namespace utils
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("RobotSkills_utils");

bool getRobotTipForFrame(const geometry_msgs::msg::PoseStamped ik_frame,
                         const planning_scene::PlanningSceneConstPtr& scene,
                         const moveit::core::JointModelGroup* jmg,
                         const moveit::core::LinkModel*& robot_link,
                         Eigen::Isometry3d& tip_in_global_frame)
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

  if (ik_frame.header.frame_id.empty())
  {
    robot_link = get_tip();
    if (!robot_link)
    {
      RCLCPP_ERROR(LOGGER, "missing ik_frame");
      return false;
    }
    tip_in_global_frame = scene->getCurrentState().getGlobalLinkTransform(robot_link);
    RCLCPP_DEBUG(LOGGER, "tip_in_global_frame: %s",
                 geometry_msgs::msg::to_yaml(tf2::toMsg(tip_in_global_frame)).c_str());
  }
  else
  {
    tf2::fromMsg(ik_frame.pose, tip_in_global_frame);

    robot_link = nullptr;
    bool found = false;
    auto ref_frame =
        scene->getCurrentState().getFrameInfo(ik_frame.header.frame_id, robot_link, found);
    if (!found && !ik_frame.header.frame_id.empty())
    {
      std::stringstream ss;
      ss << "ik_frame specified in unknown frame '" << ik_frame.header.frame_id << "'";
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
    else if (!found)
    {  // use robot link's frame as reference by default
      ref_frame = scene->getCurrentState().getGlobalLinkTransform(robot_link);
    }

    tip_in_global_frame = ref_frame * tip_in_global_frame;
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

void loadPathConstraintsFromYaml(const std::string path_constraints_yaml,
                                 moveit_msgs::msg::Constraints& path_constraints)
{
  std::string params_file = ament_index_cpp::get_package_share_directory(CURRENT_PKG) + "/config/" +
                            path_constraints_yaml;
  YAML::Node config = YAML::LoadFile(params_file);
  auto doc = config["path_constraints"];
  auto path_constraint_name = utils::getValueFromYaml<std::string>(doc, "name");

  if (path_constraint_name.empty())
  {
    std::stringstream ss;
    ss << "The name tag was empty in '" << path_constraints_yaml << "' file ";
    throw YAML::Exception(doc["name"].Mark(), ss.str());
  }
  path_constraints.name = path_constraint_name;
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
  // RWU TODO: parser PositionConstraint OrientationConstraint  VisibilityConstraint
}

}  // namespace utils
}  // namespace robot_skills
