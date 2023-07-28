#include "../include/osmVisualizer/osmVisualizer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OsmVisualizer>());
  rclcpp::shutdown();
  return 0;
}