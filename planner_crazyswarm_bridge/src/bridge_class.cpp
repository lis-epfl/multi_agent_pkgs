#include "bridge_class.hpp"

namespace planner_crazyswarm_bridge {
Bridge::Bridge() : ::rclcpp::Node("planner_crazyswarm_bridge") {
  // delcare ros parameters
  DeclareRosParameters();

  // initialize ros parameters
  InitializeRosParameters();

  // create subscriber for the position of the agent
  ::std::string pos_topic = "agent_" + ::std::to_string(id_) + "/position";
  pos_sub_ = create_subscription<::visualization_msgs::msg::Marker>(
      pos_topic, 10,
      ::std::bind(&Bridge::PositionCallback, this, ::std::placeholders::_1));

  // create subscriber for the position of the agent
  // create subscriber for the full trajectory  of the agent
  ::std::string traj_full_topic =
      "agent_" + ::std::to_string(id_) + "/traj_full";
  traj_full_sub_ =
      create_subscription<::multi_agent_planner_msgs::msg::Trajectory>(
          traj_full_topic, 10,
          ::std::bind(&Bridge::FullStateCallback, this,
                      ::std::placeholders::_1));

  // create subscriber for the position of the agent
  // create publisher for the position command
  pos_pub_ = create_publisher<::crazyflie_interfaces::msg::Position>(
      "cf" + ::std::to_string(id_) + "/cmd_position", 10);

  // create subscriber for the position of the agent
  // create publisher for the full state command
  full_state_pub_ = create_publisher<::crazyflie_interfaces::msg::FullState>(
      "cf" + ::std::to_string(id_) + "/cmd_full_state", 10);
}

void Bridge::DeclareRosParameters() {
  // declare ros parameters
  declare_parameter("id", 0);
}

void Bridge::InitializeRosParameters() {
  // intialize ros parameters
  id_ = get_parameter("id").as_int();
}

void Bridge::PositionCallback(
    const ::visualization_msgs::msg::Marker::SharedPtr marker_msg) {
  // first check if we received a trajecotry; if not proceed to send the
  // position as a position command
  if (!traj_received_) {
    // create the position message to send
    ::crazyflie_interfaces::msg::Position pos_msg;
    pos_msg.x = marker_msg->pose.position.x;
    pos_msg.y = marker_msg->pose.position.y;
    pos_msg.z = marker_msg->pose.position.z;
    pos_pub_->publish(pos_msg);
  }
}

void Bridge::FullStateCallback(
    const ::multi_agent_planner_msgs::msg::Trajectory::SharedPtr
        traj_full_msg) {
  // set the trajectory_received_ variable to true to stop sending position
  // commands
  traj_received_ = true;

  // create a full_state_msg and wait till the start of the next iteration to
  // publish the command
  ::multi_agent_planner_msgs::msg::State state = traj_full_msg->states[1];
  double dt = traj_full_msg->dt;

  ::crazyflie_interfaces::msg::FullState full_state_msg;
  full_state_msg.pose.position.x = state.position[0];
  full_state_msg.pose.position.y = state.position[1];
  full_state_msg.pose.position.z = state.position[2];
  full_state_msg.twist.linear.x = state.velocity[0];
  full_state_msg.twist.linear.y = state.velocity[1];
  full_state_msg.twist.linear.z = state.velocity[2];
  full_state_msg.acc.x = state.acceleration[0];
  full_state_msg.acc.y = state.acceleration[1];
  full_state_msg.acc.z = state.acceleration[2];

  /* sleep for the remaining time (the remainder of the modulo of the clock
  with the planning period is the sleeping time) */
  // get the clock now
  auto t_wall = ::std::chrono::high_resolution_clock::now();
  // get the time in milliseconds
  long long t_wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            t_wall.time_since_epoch())
                            .count();
  // compute the remaining time by modulo with the planning iteration
  double remaining_sleep_time_ms = (dt * 1e3) - t_wall_ms % int(dt * 1e3);

  // sleep thread
  ::std::this_thread::sleep_for(
      ::std::chrono::milliseconds(int(remaining_sleep_time_ms)));

  // publish the command
  full_state_pub_->publish(full_state_msg);
}
} // namespace planner_crazyswarm_bridge
