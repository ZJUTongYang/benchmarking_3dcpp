#include <rclcpp/rclcpp.hpp>
#include <benchmarking_3dcpp/benchmarking_3dcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Benchmarking3DCPP>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}