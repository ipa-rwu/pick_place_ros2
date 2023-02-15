#include "pick_place_app/skills/detect_aruco_marker_skill.h"

using namespace std::chrono_literals;

namespace robot_skills
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("detect_aruco_marker_skill");

DetectArucoMarkerSkill::DetectArucoMarkerSkill(rclcpp::Node::SharedPtr node,
                                               const DetectArucoMarkerSkill::Parameters& parameters)
  : node_(node), parameters_(parameters)
{
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  sub_markers_ = node_->create_subscription<aruco_msgs::msg::MarkerArray>(
      parameters_.marker_topic_name, 1,
      std::bind(&DetectArucoMarkerSkill::getMarkerCallback, this, std::placeholders::_1), options);
  RCLCPP_INFO(LOGGER, "Subscribe to : %s", parameters_.marker_topic_name.c_str());
  if_get_request_ = false;
}

DetectArucoMarkerSkill::~DetectArucoMarkerSkill() = default;

void DetectArucoMarkerSkill::getMarkerCallback(const aruco_msgs::msg::MarkerArray& msg)
{
  if (if_get_request_)
  {
    for (const auto req_marker_id : req_markers_)
    {
      RCLCPP_INFO_ONCE(LOGGER, "Check Marker [%d] pose", req_marker_id);

      auto res = std::find_if(msg.markers.begin(), msg.markers.end(),
                              [req_marker_id](const aruco_msgs::msg::Marker& x) {
                                return (int)x.id == req_marker_id;
                              });
      if (res != msg.markers.end())
      {
        auto marker_pose_vec = res_markers_.find(req_marker_id);
        geometry_msgs::msg::PoseStamped marker_posestamped;
        marker_posestamped.header = res->header;
        marker_posestamped.pose = res->pose.pose;

        if (marker_pose_vec == res_markers_.end())
        {
          std::vector<geometry_msgs::msg::PoseStamped> tmp;

          tmp.push_back(marker_posestamped);
          RCLCPP_INFO(LOGGER, "Get marker ID:[%d]: %s", res->id,
                      geometry_msgs::msg::to_yaml(res->pose.pose).c_str());
          res_markers_.insert(std::make_pair(req_marker_id, tmp));
        }
        else
        {
          res_markers_[req_marker_id].push_back(marker_posestamped);
        }
      }
    }
  }
}

bool DetectArucoMarkerSkill::getArucoPosestamps(
    const std::vector<int>& marker_ids, std::map<int, geometry_msgs::msg::PoseStamped>& result)
{
  if_get_request_ = true;
  auto start_time = std::chrono::steady_clock::now();

  res_markers_.clear();
  req_markers_ = marker_ids;
  RCLCPP_INFO(LOGGER, "Get detect Aruco marker request!");

  while ((std::chrono::steady_clock::now() - start_time) <
         std::chrono::seconds(parameters_.find_marker_timeout_seconds))
  {
    if_get_request_ = true;
    rclcpp::sleep_for(1000ms);
    RCLCPP_INFO(LOGGER, "Waiting for detection");
  }
  if_get_request_ = false;

  for (const auto req_marker_id : marker_ids)
  {
    auto poses = res_markers_.find(req_marker_id);

    if (poses != res_markers_.end() && poses->second.size() > 0)
    {
      auto all_poses = poses->second;
      geometry_msgs::msg::PoseStamped res_marker;
      res_marker.header = all_poses.at(0).header;
      res_marker.pose.position.x =
          std::accumulate(all_poses.begin(), all_poses.end(), 0.0000,
                          [](double sum, const geometry_msgs::msg::PoseStamped& curr) {
                            return sum + curr.pose.position.x;
                          }) /
          all_poses.size();

      res_marker.pose.position.y =
          std::accumulate(all_poses.begin(), all_poses.end(), 0.0000,
                          [](double sum, const geometry_msgs::msg::PoseStamped& curr) {
                            return sum + curr.pose.position.y;
                          }) /
          all_poses.size();
      res_marker.pose.position.z =
          std::accumulate(all_poses.begin(), all_poses.end(), 0.0000,
                          [](double sum, const geometry_msgs::msg::PoseStamped& curr) {
                            return sum + curr.pose.position.z;
                          }) /
          all_poses.size();
      RCLCPP_INFO(LOGGER, "get marker [%d]: pose: %s", req_marker_id,
                  geometry_msgs::msg::to_yaml(res_marker).c_str());
      result.insert(std::make_pair(req_marker_id, res_marker));
    }
    else
    {
      return false;
    }
  }
  return true;
}

}  // namespace robot_skills
