#include "pick_place_app/skills/modify_planning_scene_skill.h"
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace robot_skills
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("modify_planning_scene_skill");

ModifyPlanningSceneSkill::ModifyPlanningSceneSkill(
    rclcpp::Node::SharedPtr node, moveit::planning_interface::PlanningSceneInterfacePtr psi)
  : node_(node), psi_(psi)
{
}

ModifyPlanningSceneSkill::~ModifyPlanningSceneSkill() = default;

moveit_msgs::msg::CollisionObject
ModifyPlanningSceneSkill::createBox(const std::string object_name, const std::string frame_id,
                                    const geometry_msgs::msg::Pose& pose,
                                    const std::vector<double>& box_shape)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = frame_id;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions.resize(
      geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::BOX>());
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::BOX_X) = box_shape.at(0);
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::BOX_Y) = box_shape.at(1);
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::BOX_Z) = box_shape.at(2);

  object.pose = pose;
  return object;
}

void ModifyPlanningSceneSkill::addObject(moveit_msgs::msg::CollisionObject& object, bool invert)
{
  if (!invert)
  {
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi_->applyCollisionObject(object);
  }
  else
    psi_->removeCollisionObjects({ object.id });
}

void ModifyPlanningSceneSkill::allowCollisions(const Names& first, const Names& second, bool allow)
{
  collision_matrix_edits_.push_back(CollisionMatrixPairs({ first, second, allow }));
}

void ModifyPlanningSceneSkill::allowCollisions(planning_scene::PlanningScenePtr& scene,
                                               const CollisionMatrixPairs& pairs, bool invert)
{
  collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();
  bool allow = invert ? !pairs.allow : pairs.allow;
  if (pairs.second.empty())
  {
    for (const auto& name : pairs.first)
      acm.setEntry(name, allow);
  }
  else
    acm.setEntry(pairs.first, pairs.second, allow);
}

void ModifyPlanningSceneSkill::attachObject(planning_scene::PlanningScenePtr& scene,
                                            const std::string& group,
                                            moveit_msgs::msg::CollisionObject& object,
                                            const std::string& attach_link, bool invert)
{
  const moveit::core::RobotModelConstPtr& robot_model = scene->getRobotModel();
  moveit::planning_interface::MoveGroupInterface move_group(node_, group);

  move_group.attachObject(object.id, attach_link);
  if (invert)
  {
    move_group.detachObject(object.id);
  }
}

}  // namespace robot_skills
