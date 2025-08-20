#include "estimate_ground_surface.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<ObstacleDetection>((node_options)));
  rclcpp::shutdown();
  return 0;
}