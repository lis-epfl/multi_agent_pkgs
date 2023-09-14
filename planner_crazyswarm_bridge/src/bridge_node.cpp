#include "bridge_class.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<planner_crazyswarm_bridge::Bridge>());
  rclcpp::shutdown();
}
