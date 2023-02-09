#ifndef ROBOT_SKILLS__DETECT_ARUCO_MARKER_SKILL_H_
#define ROBOT_SKILLS__DETECT_ARUCO_MARKER_SKILL_H_

#include <rclcpp/rclcpp.hpp>
#include "aruco_msgs/msg/marker_array.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>

namespace robot_skills
{
class DetectArucoMarkerSkill
{
public:
  /**
   * @brief
   *
   */
  RCLCPP_SMART_PTR_DEFINITIONS(DetectArucoMarkerSkill)

  struct Parameters
  {
    std::string marker_topic_name;
    int find_marker_timeout_seconds = 10;
    void loadParameters(const rclcpp::Node::SharedPtr& node);
  };

  DetectArucoMarkerSkill(rclcpp::Node::SharedPtr node,
                         const DetectArucoMarkerSkill::Parameters& parameters);

  ~DetectArucoMarkerSkill();

  double getMovingAvg(const std::vector<double>& vec, std::size_t requested_window_width);
  bool getArucoPosestamps(const std::vector<int>& marker_ids,
                          std::map<int, geometry_msgs::msg::PoseStamped>& result);

protected:
  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr sub_markers_;

  void getMarkerCallback(const aruco_msgs::msg::MarkerArray& msg);

private:
  rclcpp::Node::SharedPtr node_;
  DetectArucoMarkerSkill::Parameters parameters_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  bool if_get_request_;
  std::vector<int> req_markers_;
  std::map<int, std::vector<geometry_msgs::msg::PoseStamped>> res_markers_;
};

}  // namespace robot_skills

#endif
