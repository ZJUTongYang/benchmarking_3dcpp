#include <rclcpp/rclcpp.hpp>
#include <benchmarking_3dcpp/benchmarking_3dcpp.hpp>
#include <benchmarking_3dcpp/alg/nuc.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Benchmarking3DCPP>();

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(),
    std::thread::hardware_concurrency()
  );

  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
}