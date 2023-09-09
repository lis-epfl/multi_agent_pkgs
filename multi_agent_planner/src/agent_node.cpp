#include "agent_class.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_agent_planner::Agent>());
  rclcpp::shutdown();
}
