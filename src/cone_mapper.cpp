#include "cone_mapper/cone_mapper.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>

namespace cone_mapper
{

ConeMapper::ConeMapper()
: Node("cone_mapper")
{
  target_frame_ = declare_parameter<std::string>("target_frame", "odom");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  marker_sub_ = create_subscription<visualization_msgs::msg::Marker>(
    "/cones", 10,
    std::bind(&ConeMapper::markerCallback, this, std::placeholders::_1));

  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
    "/cones_mapped", 10);

  RCLCPP_INFO(get_logger(), "ConeMapper node started. Target frame: %s", target_frame_.c_str());
}

void ConeMapper::markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
  visualization_msgs::msg::Marker transformed_marker = *msg;
  transformed_marker.points.clear();  // clear points before adding transformed ones
  transformed_marker.header.frame_id = target_frame_;

  for (const auto & point : msg->points)
  {
    try
    {
      geometry_msgs::msg::Point transformed_point = transformPoint(
        point,
        msg->header.frame_id,
        target_frame_,
        msg->header.stamp);

      transformed_marker.points.push_back(transformed_point);
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(get_logger(), "Transform failed for a point: %s", ex.what());
    }
  }

  marker_pub_->publish(transformed_marker);
}

geometry_msgs::msg::Point ConeMapper::transformPoint(
  const geometry_msgs::msg::Point & point,
  const std::string & from_frame,
  const std::string & to_frame,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PointStamped point_in, point_out;
  point_in.header.frame_id = from_frame;
  point_in.header.stamp = stamp;
  point_in.point = point;

  point_out = tf_buffer_->transform(point_in, to_frame, tf2::durationFromSec(0.5));
  return point_out.point;
}

}  // namespace cone_mapper
