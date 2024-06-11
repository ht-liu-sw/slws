#include <cstdio>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

#include "sensor_process_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<sensor_process_node>("sensor_process");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
