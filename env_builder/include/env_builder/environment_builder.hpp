#ifndef ENV_BUILDER_ENVIRONMENT_BUILDER_CLASS_H_
#define ENV_BUILDER_ENVIRONMENT_BUILDER_CLASS_H_

#include "rclcpp/rclcpp.hpp"
#include "voxel_grid.hpp"
#include <env_builder_msgs/msg/voxel_grid_stamped.hpp>
#include <env_builder_msgs/srv/get_voxel_grid.hpp>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdlib.h>
#include <string>

namespace env_builder {
class EnvironmentBuilder : public ::rclcpp::Node {
public:
  EnvironmentBuilder();

private:
  /*-------------- methods ---------------*/
  // declare ros parameters
  void DeclareRosParameters();

  // initialize ros parameters
  void InitializeRosParameters();

  // create an empty voxel grid with unknown voxels
  void CreateEmptyVoxelGrid();

  // add the randomly generated obstacles to the voxel grid
  void AddObstacles();

  // inflate the obstacles by inflation_dist_
  void InflateObstacles();

  // generate potential field using potential_dist_ and potential_pow_
  void CreatePotentialField();

  // save obstacle pointcloud to file
  void SaveObstacles();

  // create the environment pointcloud to publish it regularly
  void CreateEnvironmentPointCloud();

  // create the environment voxel grid to publish it regularly
  void CreateEnvironmentVoxelGrid();

  // function that publishes the environment pointcloud at constant frequency
  void TimerCallbackEnvironmentPC();

  // function that publishes the environment voxel grid at constant frequency
  void TimerCallbackEnvironmentVG();

  // generate the sub-voxel grid from the main one using the robot position and
  // its sensing range
  ::env_builder_msgs::msg::VoxelGridStamped
  GenerateVoxelGridMSG(::std::array<double, 3> &position,
                       ::std::array<double, 3> &range);

  // voxel grid get service
  void GetVoxelGridService(
      const ::std::shared_ptr<::env_builder_msgs::srv::GetVoxelGrid::Request>
          request,
      ::std::shared_ptr<::env_builder_msgs::srv::GetVoxelGrid::Response>);

  /*-------------- member variables ---------------*/
  // ROS parameters
  ::std::vector<double> origin_grid_;
  ::std::vector<double> dimension_grid_;
  double vox_size_;
  // if true, initialize grid to be free instead of unknown
  double free_grid_;
  // if true wirte obstacle positions and pointcloud to csv file
  bool save_obstacles_;
  // voxel grid publishing period
  double publish_period_;

  // obstacles parameters
  bool multi_obst_size_;
  bool multi_obst_position_;
  ::std::vector<double> range_obst_;
  ::std::vector<double> origin_obst_;
  ::std::vector<double> size_obst_;
  ::std::vector<double> size_obst_vec_;
  ::std::vector<double> position_obst_vec_;
  int n_obst_;
  int rand_seed_;

  // environment message params
  ::std::string env_pc_topic_;
  ::std::string env_vg_topic_;
  ::std::string env_pc_frame_;

  // member variables
  ::voxel_grid_util::VoxelGrid::Ptr voxel_grid_shared_ptr_;

  // variables to publish the environment pointcloud at constant frequency
  ::std::shared_ptr<::sensor_msgs::msg::PointCloud2> env_pc_msg_;
  ::rclcpp::TimerBase::SharedPtr env_pub_timer_;
  ::rclcpp::Publisher<::sensor_msgs::msg::PointCloud2>::SharedPtr env_pc_pub_;

  // variable to publish the environement voxel grid at constant frequency
  ::env_builder_msgs::msg::VoxelGridStamped voxel_grid_stamped_msg_;
  ::rclcpp::TimerBase::SharedPtr voxel_grid_timer_;
  ::rclcpp::Publisher<::env_builder_msgs::msg::VoxelGridStamped>::SharedPtr
      voxel_grid_pub_;

  // service for getting a sub-voxel grid from the environment voxel grid that
  // corresponds to the position and sensing range of the robot
  ::rclcpp::Service<::env_builder_msgs::srv::GetVoxelGrid>::SharedPtr
      voxel_grid_service_;
  ::std::string get_grid_service_name_;
};

} // namespace env_builder

#endif // ENV_BUILDER_ENVIRONMENT_BUILDER_CLASS_H_
