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

namespace robot_skills
{
namespace utils
{
bool getRobotTipForFrame(const geometry_msgs::msg::PoseStamped ik_frame,
                         const planning_scene::PlanningSceneConstPtr& scene,
                         const moveit::core::JointModelGroup* jmg,
                         const moveit::core::LinkModel*& robot_link,
                         Eigen::Isometry3d& tip_in_global_frame);
}
}  // namespace robot_skills

#endif
