#ifndef CONE_MAPPER_HPP_
#define CONE_MAPPER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace cone_mapper
{

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

  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace cone_mapper

#endif  // CONE_MAPPER_HPP_
