#include "pick_place_app/skills/utils.h"

namespace robot_skills
{
namespace utils
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("RobotSkills_utils");

bool getRobotTipForFrame(const std::string ik_frame, const planning_scene::PlanningScene& scene,
                         const moveit::core::JointModelGroup* jmg,
                         const moveit::core::LinkModel*& robot_link,
                         Eigen::Isometry3d& tip_in_global_frame)
{
  RCLCPP_INFO(LOGGER, "getRobotTipForFrame: ik_frame: %s", ik_frame.c_str());
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

  robot_link = get_tip();
  if (!robot_link)
  {
    RCLCPP_ERROR(LOGGER, "missing ik_frame");
    return false;
  }
  tip_in_global_frame = scene.getCurrentState().getGlobalLinkTransform(robot_link);

  return true;
}
}  // namespace utils
}  // namespace robot_skills
