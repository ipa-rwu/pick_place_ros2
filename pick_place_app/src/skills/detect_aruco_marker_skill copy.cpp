#include "pick_place_app/skills/detect_aruco_marker_skill.h"
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace robot_skills
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("modify_planning_scene_skill");

void DetectArucoMarkerSkill::Parameters::loadParameters(const rclcpp::Node::SharedPtr& node)
{
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(node, "marker_topic_name",
                                     marker_topic_name);  //
  RCLCPP_INFO(LOGGER, "param: %s", marker_topic_name.c_str());

  rosparam_shortcuts::shutdownIfError(errors);
}

DetectArucoMarkerSkill::DetectArucoMarkerSkill(rclcpp::Node::SharedPtr node,
                                               const DetectArucoMarkerSkill::Parameters& parameters)
  : node_(node), parameters_(parameters)
{
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub_markers_ = node_->create_subscription<aruco_msgs::msg::MarkerArray>(
      parameters_.marker_topic_name, 10,
      std::bind(&DetectArucoMarkerSkill::getMarkerCallback, this, std::placeholders::_1));

  get_aruco_posestamp_service_ = node_->create_service<pick_place_msgs::srv::GetArucoPosestamp>(
      parameters_.get_aruco_pose_service_name,
      std::bind(&DetectArucoMarkerSkill::getArucoPosestampCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3),
      rmw_qos_profile_services_default, callback_group_);
}

DetectArucoMarkerSkill::~DetectArucoMarkerSkill() = default;

void DetectArucoMarkerSkill::getMarkerCallback(
    const aruco_msgs::msg::MarkerArray::ConstSharedPtr& msg)
{
  if (if_get_request_)
  {
    for (const auto req_marker_id : req_markers_)
    {
      auto res = std::find_if(msg.get()->markers.begin(), msg.get()->markers.end(),
                              [&](const aruco_msgs::msg::Marker& x) {
                                return (int)x.id == req_marker_id;
                              });
      auto poses = res_markers_.find(req_marker_id);
      std::vector<aruco_msgs::msg::Marker> tmp;
      if (poses == res_markers_.end())
      {
        tmp.push_back(*res);
        res_markers_.insert(std::make_pair(req_marker_id, tmp));
      }
      else
      {
        tmp = poses->second;
        tmp.push_back(*res);
        res_markers_.insert(std::make_pair(req_marker_id, tmp));
      }
    }
  }
}

void DetectArucoMarkerSkill::getArucoPosestampCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<pick_place_msgs::srv::GetArucoPosestamp::Request> request,
    std::shared_ptr<pick_place_msgs::srv::GetArucoPosestamp::Response> response)
{
  if_get_request_ = true;
  auto start_time = std::chrono::steady_clock::now();
  req_markers_.clear();
  req_markers_ = request->marker_ids;
  while ((std::chrono::steady_clock::now() - start_time) < parameters_.timeout)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  if_get_request_ = false;
  req_markers_.clear();

  for (const auto req_marker_id : req_markers_)
  {
    auto poses = res_markers_.find(req_marker_id);
    auto tmp = poses->second;
    if (poses != res_markers_.end() && tmp.size() > 0)
    {
      aruco_msgs::msg::Marker res_marker;
      res_marker.id = req_marker_id;
      res_marker.pose.pose.position.x = std::accumulate(
          tmp.begin(), tmp.end(), 0, [](const double& sum_x, aruco_msgs::msg::Marker& b) {
            return sum_x + b.pose.pose.position.x;
          });
      res_marker.pose.pose.position.y = std::accumulate(
          tmp.begin(), tmp.end(), 0, [](const double& sum_y, aruco_msgs::msg::Marker& b) {
            return sum_y + b.pose.pose.position.y;
          });
      res_marker.pose.pose.position.z = std::accumulate(
          tmp.begin(), tmp.end(), 0, [](const double& sum_z, aruco_msgs::msg::Marker& b) {
            return sum_z + b.pose.pose.position.z;
          });
      RCLCPP_INFO(LOGGER, "get marker [%d]: pose: %s", req_marker_id,
                  geometry_msgs::msg::to_yaml(res_marker.pose.pose).c_str());
      response->markers.push_back(res_marker);
    }
    else
    {
      response->result = false;
    }
  }
  response->result = true;
}

}  // namespace robot_skills
