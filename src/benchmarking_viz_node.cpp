#include <rclcpp/rclcpp.hpp>
#include <benchmarking_3dcpp/viz/benchmarking_viz.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BenchmarkingViz>();

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(),
    std::thread::hardware_concurrency()
  );

  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
}