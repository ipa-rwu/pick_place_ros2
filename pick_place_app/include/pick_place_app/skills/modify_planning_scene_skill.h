#ifndef ROBOT_SKILLS__MODIFY_PLANNING_SCENE_SKILL_H_
#define ROBOT_SKILLS__MODIFY_PLANNING_SCENE_SKILL_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <map>
#include <memory>

namespace robot_skills
{
class ModifyPlanningSceneSkill
{
public:
  /**
   * @brief
   *
   */
  RCLCPP_SMART_PTR_DEFINITIONS(ModifyPlanningSceneSkill)
  using Names = std::vector<std::string>;

  ModifyPlanningSceneSkill(rclcpp::Node::SharedPtr node,
                           moveit::planning_interface::PlanningSceneInterfacePtr psi);

  ~ModifyPlanningSceneSkill();

  moveit_msgs::msg::CollisionObject createBox(const std::string object_name,
                                              const std::string frame_id,
                                              const geometry_msgs::msg::Pose& pose,
                                              const std::vector<double>& box_shape);

  void addObject(moveit_msgs::msg::CollisionObject& object, bool invert = false);

  void attachObject(planning_scene::PlanningScenePtr& scene, const std::string& group,
                    moveit_msgs::msg::CollisionObject& object, const std::string& attach_link,
                    bool invert);

  /// allow / forbid collisions for each combination of pairs in first and second lists
  void allowCollisions(const Names& first, const Names& second, bool allow);
  /// allow / forbid collisions for pair (first, second)
  void allowCollisions(const std::string& first, const std::string& second, bool allow)
  {
    allowCollisions(Names{ first }, Names{ second }, allow);
  }
  /// allow / forbid all collisions for given object
  void allowCollisions(const std::string& object, bool allow)
  {
    allowCollisions(Names({ object }), Names(), allow);
  }

protected:
  // list of objects to mutually
  struct CollisionMatrixPairs
  {
    Names first;
    Names second;
    bool allow;
  };
  std::list<CollisionMatrixPairs> collision_matrix_edits_;

  void allowCollisions(planning_scene::PlanningScenePtr& scene, const CollisionMatrixPairs& pairs,
                       bool invert);

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::PlanningSceneInterfacePtr psi_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client_;

  // list of objects to attach (true) / detach (false) to a given link
  std::map<std::string, std::pair<Names, bool>> attach_objects_;
  // list of objects to add / remove to the planning scene
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects_;
};

}  // namespace robot_skills

#endif
