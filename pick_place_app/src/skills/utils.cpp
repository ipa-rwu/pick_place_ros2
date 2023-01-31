#include "pick_place_app/skills/utils.h"

namespace robot_skills
{
namespace utils
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("RobotSkills_utils");

bool getRobotTipForFrame(const geometry_msgs::msg::PoseStamped ik_frame,
                         const planning_scene::PlanningScene& scene,
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
    RCLCPP_INFO(LOGGER, "getRobotTipForFrame: get tip: %s", tips[0]->getName().c_str());
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
    tip_in_global_frame = scene.getCurrentState().getGlobalLinkTransform(robot_link);
    RCLCPP_INFO(LOGGER, "tip_in_global_frame: %s",
                geometry_msgs::msg::to_yaml(tf2::toMsg(tip_in_global_frame)).c_str());
  }
  else
  {
    tf2::fromMsg(ik_frame.pose, tip_in_global_frame);

    robot_link = nullptr;
    bool found = false;
    auto ref_frame =
        scene.getCurrentState().getFrameInfo(ik_frame.header.frame_id, robot_link, found);
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
      ref_frame = scene.getCurrentState().getGlobalLinkTransform(robot_link);
    }

    tip_in_global_frame = ref_frame * tip_in_global_frame;
  }

  return true;
}
}  // namespace utils
}  // namespace robot_skills
