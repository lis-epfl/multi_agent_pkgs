#include "agent_class.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Create your node
  auto agent_node = std::make_shared<multi_agent_planner::Agent>();

  // Create a multithreaded executor
  rclcpp::executors::MultiThreadedExecutor executor;

  // Add node to executor
  executor.add_node(agent_node);

  // Start spinning
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
