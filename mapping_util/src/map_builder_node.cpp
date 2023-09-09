#include "map_builder.hpp"

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mapping_util::MapBuilder>());
  rclcpp::shutdown();
}
