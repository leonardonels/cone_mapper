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
    "/cone_map", 10);

  RCLCPP_INFO(get_logger(), "ConeMapper node started. Target frame: %s", target_frame_.c_str());
}

void ConeMapper::markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
  std::vector<geometry_msgs::msg::Point> transformed_points;

  for (const auto & point : msg->points)
  {
    try
    {
      geometry_msgs::msg::Point transformed_point = transformPoint(
        point,
        msg->header.frame_id,
        target_frame_,
        msg->header.stamp);

      transformed_points.push_back(transformed_point);
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(get_logger(), "Transform failed for a point: %s", ex.what());
    }
  }

  updateTrackedCones(transformed_points);

  // Publish stable cones as marker
  visualization_msgs::msg::Marker cone_map;
  cone_map.header.frame_id = target_frame_;
  cone_map.header.stamp = this->now();
  cone_map.ns = "cone_map";
  cone_map.id = 0;
  cone_map.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  cone_map.action = visualization_msgs::msg::Marker::ADD;
  cone_map.scale.x = 0.2;
  cone_map.scale.y = 0.2;
  cone_map.scale.z = 0.2;
  cone_map.color.r = 1.0f;
  cone_map.color.g = 0.5f;
  cone_map.color.b = 0.0f;
  cone_map.color.a = 1.0f;

  for (const auto & cone : tracked_cones_)
  {
    if (cone.observation_count >= min_observations_)
    {
      cone_map.points.push_back(cone.position);
    }
  }

  marker_pub_->publish(cone_map);
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

void ConeMapper::updateTrackedCones(const std::vector<geometry_msgs::msg::Point> & detected_cones)
{
  for (const auto & detected : detected_cones)
  {
    bool matched = false;
    for (auto & tracked : tracked_cones_)
    {
      if (distance(detected, tracked.position) < association_distance_)
      {
        //tracked.position = average(tracked.position, detected);
        tracked.observation_count++;
        matched = true;
        break;
      }
    }

    if (!matched)
    {
      tracked_cones_.push_back({detected, 1});
    }
  }
}

double ConeMapper::distance(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

geometry_msgs::msg::Point ConeMapper::average(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  geometry_msgs::msg::Point avg;
  avg.x = (p1.x + p2.x) / 2.0;
  avg.y = (p1.y + p2.y) / 2.0;
  avg.z = (p1.z + p2.z) / 2.0;
  return avg;
}

}  // namespace cone_mapper
