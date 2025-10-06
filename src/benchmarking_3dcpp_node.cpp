#include <rclcpp/rclcpp.hpp>
#include <benchmarking_3dcpp/benchmarking_3dcpp.hpp>
#include <benchmarking_3dcpp/alg/nuc.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Benchmarking3DCPP>();
  // auto nuc_client = std::make_shared<NUCClient>();

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(),
    std::thread::hardware_concurrency()
  );

  executor.add_node(node);
  // executor.add_node(nuc_client);

  executor.spin();


  rclcpp::shutdown();
}