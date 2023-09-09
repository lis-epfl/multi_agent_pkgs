#include "environment_builder.hpp"

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<env_builder::EnvironmentBuilder>());
  rclcpp::shutdown();
}
