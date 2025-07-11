#ifndef CONE_MAPPER_HPP_
#define CONE_MAPPER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace cone_mapper
{

struct TrackedCone
{
  geometry_msgs::msg::Point position;
  int observation_count;
};

class ConeMapper : public rclcpp::Node
{
public:
  ConeMapper();

private:
  void markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
  geometry_msgs::msg::Point transformPoint(
    const geometry_msgs::msg::Point & point,
    const std::string & from_frame,
    const std::string & to_frame,
    const rclcpp::Time & stamp);

  void updateTrackedCones(const std::vector<geometry_msgs::msg::Point> & detected_cones);
  double distance(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);
  geometry_msgs::msg::Point average(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::vector<TrackedCone> tracked_cones_;
  const double association_distance_ = 0.3;
  const int min_observations_ = 5;  // minimum for reliable cone
};

}  // namespace cone_mapper

#endif  // CONE_MAPPER_HPP_
