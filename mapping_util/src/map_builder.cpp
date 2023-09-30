#include "map_builder.hpp"

namespace mapping_util {
MapBuilder::MapBuilder() : ::rclcpp::Node("map_builder") {
  // declare environment parameters
  DeclareRosParameters();

  // initialize parameters
  InitializeRosParameters();

  // resize current position
  pos_curr_.resize(3);
  
  // create environment voxel grid subscriber
  voxel_grid_sub_ =
      create_subscription<::env_builder_msgs::msg::VoxelGridStamped>(
          env_vg_topic_, 10,
          ::std::bind(&MapBuilder::EnvironmentVoxelGridCallback, this,
                      ::std::placeholders::_1));

  // create agent position subscriber
  agent_frame_ = "agent_" + ::std::to_string(id_);
  tf_buffer_ = ::std::make_shared<::tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
      ::std::make_shared<::tf2_ros::TransformListener>(*tf_buffer_, this);

  tf_subscriber_ = this->create_subscription<::tf2_msgs::msg::TFMessage>(
      "/tf", 10,
      ::std::bind(&MapBuilder::TfCallback, this, ::std::placeholders::_1));

  // create voxel grid publisher
  ::std::string vg_pub_topic = "agent_" + ::std::to_string(id_) + "/voxel_grid";
  voxel_grid_pub_ = create_publisher<::env_builder_msgs::msg::VoxelGridStamped>(
      vg_pub_topic, 10);
}

void MapBuilder::DeclareRosParameters() {
  declare_parameter("env_vg_topic", "env_builder_node/environment_voxel_grid");
  declare_parameter("id", 0);
  declare_parameter("voxel_grid_range", ::std::vector<double>(3, 10.0));
  declare_parameter("world_frame", "world");
  declare_parameter("free_grid", true);
}

void MapBuilder::InitializeRosParameters() {
  env_vg_topic_ = get_parameter("env_vg_topic").as_string();
  id_ = get_parameter("id").as_int();
  voxel_grid_range_ = get_parameter("voxel_grid_range").as_double_array();
  world_frame_ = get_parameter("world_frame").as_string();
  free_grid_ = get_parameter("free_grid").as_bool();
}

void MapBuilder::EnvironmentVoxelGridCallback(
    const ::env_builder_msgs::msg::VoxelGridStamped::SharedPtr vg_msg) {
  // first check if we received the first position
  if (pos_curr_.size() > 0) {
    double voxel_size = vg_msg->voxel_grid.voxel_size;
    voxel_grid_curr_.voxel_grid.voxel_size = voxel_size;
    voxel_grid_curr_.voxel_grid.potential_dist =
        vg_msg->voxel_grid.potential_dist;
    voxel_grid_curr_.voxel_grid.potential_pow =
        vg_msg->voxel_grid.potential_pow;

    // find the origin of the grid
    ::std::array<float, 3> origin_grid = vg_msg->voxel_grid.origin;
    ::std::array<float, 3> origin;
    pos_mutex_.lock();
    origin[0] = (pos_curr_[0] - voxel_grid_range_[0] / 2);
    origin[1] = (pos_curr_[1] - voxel_grid_range_[1] / 2);
    origin[2] = (pos_curr_[2] - voxel_grid_range_[2] / 2);
    pos_mutex_.unlock();
    origin[0] = floor((origin[0] - origin_grid[0]) / voxel_size) * voxel_size +
                origin_grid[0];
    origin[1] = floor((origin[1] - origin_grid[1]) / voxel_size) * voxel_size +
                origin_grid[1];
    origin[2] = floor((origin[2] - origin_grid[2]) / voxel_size) * voxel_size +
                origin_grid[2];
    voxel_grid_curr_.voxel_grid.origin = origin;

    // find the range in integer dimensions
    ::std::array<uint32_t, 3> dim;
    dim[0] = floor(voxel_grid_range_[0] / voxel_size);
    dim[1] = floor(voxel_grid_range_[1] / voxel_size);
    dim[2] = floor(voxel_grid_range_[2] / voxel_size);

    voxel_grid_curr_.voxel_grid.dimension = dim;

    // find the starting index
    ::std::vector<int> start_idx;
    start_idx.push_back(
        ::std::round((origin[0] - origin_grid[0]) / voxel_size));
    start_idx.push_back(
        ::std::round((origin[1] - origin_grid[1]) / voxel_size));
    start_idx.push_back(
        ::std::round((origin[2] - origin_grid[2]) / voxel_size));

    // generate the sub voxel grid from the environment
    voxel_grid_curr_.voxel_grid.data.resize(dim[0] * dim[1] * dim[2]);
    ::voxel_grid_util::VoxelGrid vg =
        ::mapping_util::ConvertVGMsgToVGUtil(vg_msg->voxel_grid);

    for (int i = start_idx[0]; i < start_idx[0] + int(dim[0]); i++) {
      for (int j = start_idx[1]; j < start_idx[1] + int(dim[1]); j++) {
        for (int k = start_idx[2]; k < start_idx[2] + int(dim[2]); k++) {
          int i_msg = i - start_idx[0];
          int j_msg = j - start_idx[1];
          int k_msg = k - start_idx[2];
          int idx = i_msg + dim[0] * j_msg + dim[0] * dim[1] * k_msg;
          voxel_grid_curr_.voxel_grid.data[idx] =
              vg.GetVoxelInt(::Eigen::Vector3i(i, j, k));
          if (free_grid_) {
            if (voxel_grid_curr_.voxel_grid.data[idx] == -1) {
              voxel_grid_curr_.voxel_grid.data[idx] = 0;
            }
          }
        }
      }
    }

    voxel_grid_curr_.header.stamp = now();
    voxel_grid_curr_.header.frame_id = world_frame_;

    voxel_grid_pub_->publish(voxel_grid_curr_);
  }
}

void MapBuilder::TfCallback(const ::tf2_msgs::msg::TFMessage::SharedPtr msg) {
  for (const auto &transform_stamped : msg->transforms) {
    if (transform_stamped.header.frame_id == world_frame_ &&
        transform_stamped.child_frame_id == agent_frame_) {
      // get the position from the transform
      const geometry_msgs::msg::Transform &transform =
          transform_stamped.transform;
      pos_curr_[0] = transform.translation.x;
      pos_curr_[1] = transform.translation.y;
      pos_curr_[2] = transform.translation.z;
    }
  }
}

::voxel_grid_util::VoxelGrid
ConvertVGMsgToVGUtil(::env_builder_msgs::msg::VoxelGrid &vg_msg) {
  // get the origin
  ::Eigen::Vector3d origin(vg_msg.origin[0], vg_msg.origin[1],
                           vg_msg.origin[2]);

  // get the dimensions
  ::Eigen::Vector3i dimension(vg_msg.dimension[0], vg_msg.dimension[1],
                              vg_msg.dimension[2]);

  // create voxel grid object
  ::voxel_grid_util::VoxelGrid vg(origin, dimension, vg_msg.voxel_size,
                                  vg_msg.data);

  // return the voxel grid object
  return vg;
}

::env_builder_msgs::msg::VoxelGrid
ConvertVGUtilToVGMsg(::voxel_grid_util::VoxelGrid &vg) {
  ::env_builder_msgs::msg::VoxelGrid vg_msg;

  // get the origin
  ::Eigen::Vector3d origin = vg.GetOrigin();
  vg_msg.origin[0] = origin[0];
  vg_msg.origin[1] = origin[1];
  vg_msg.origin[2] = origin[2];

  // get dimensions
  ::Eigen::Vector3i dim = vg.GetDim();
  vg_msg.dimension[0] = dim[0];
  vg_msg.dimension[1] = dim[1];
  vg_msg.dimension[2] = dim[2];

  // get voxel size
  vg_msg.voxel_size = vg.GetVoxSize();

  // get data
  vg_msg.data = vg.GetData();

  // return voxel grid message
  return vg_msg;
}
} // namespace mapping_util
