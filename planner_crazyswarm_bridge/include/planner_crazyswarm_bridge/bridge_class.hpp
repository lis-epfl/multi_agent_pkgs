#ifndef PLANNER_CRAZYSWARM_BRIDGE_CLASS_H_
#define PLANNER_CRAZYSWARM_BRIDGE_CLASS_H_

#include "crazyflie_interfaces/msg/full_state.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "multi_agent_planner_msgs/msg/state.hpp"
#include "multi_agent_planner_msgs/msg/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <visualization_msgs/msg/marker.hpp>

namespace planner_crazyswarm_bridge {
class Bridge : public ::rclcpp::Node {
public:
  Bridge();

private:
  /*-------------- methods ---------------*/
  // declare ros parameters
  void DeclareRosParameters();

  // initialize ros parameters
  void InitializeRosParameters();

  // callback function for the position
  void PositionCallback(
      const ::visualization_msgs::msg::Marker::SharedPtr marker_msg);

  // callback function for the trajectory
  void
  FullStateCallback(const ::multi_agent_planner_msgs::msg::Trajectory::SharedPtr
                        traj_full_msg);

  /*-------------- member variables ---------------*/
  // ROS parameters
  int id_;

  // variable to indicate if we received the first trajectory. In that case,
  // stop sending the position command every time we receive a new position
  bool traj_received_ = false;

  // subscriber to get the position of the agent
  ::rclcpp::Subscription<::visualization_msgs::msg::Marker>::SharedPtr pos_sub_;

  // subscriber to get the position of the agent
  ::rclcpp::Subscription<::multi_agent_planner_msgs::msg::Trajectory>::SharedPtr
      traj_full_sub_;

  // publisher to publish the initial position while waiting for the trajectory
  // to arrive
  ::rclcpp::Publisher<::crazyflie_interfaces::msg::Position>::SharedPtr
      pos_pub_;

  // publisher to publish the full state command once we start receiving the
  // full trajectory
  ::rclcpp::Publisher<::crazyflie_interfaces::msg::FullState>::SharedPtr
      full_state_pub_;
};
} // namespace planner_crazyswarm_bridge
#endif
