#include "cone_mapper/cone_mapper.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cone_mapper::ConeMapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
