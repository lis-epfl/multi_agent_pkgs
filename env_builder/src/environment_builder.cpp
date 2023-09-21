#include "environment_builder.hpp"

namespace env_builder {
EnvironmentBuilder::EnvironmentBuilder()
    : ::rclcpp::Node("environment_builder") {

  // declare environment parameters
  DeclareRosParameters();

  // initialize parameters
  InitializeRosParameters();

  // create voxel grid
  CreateEmptyVoxelGrid();

  // add obstacles to the voxel grid
  AddObstacles();

  // inflate the obstacles by inflation_dist_
  InflateObstacles();

  // generate potential field from potential_dist_ and potential_pow_
  CreatePotentialField();

  // save obstacle positions and pointcloud to file if save_obstacles_ is true
  if (save_obstacles_) {
    SaveObstacles();
  }

  // create pointcloud from voxel grid
  CreateEnvironmentPointCloud();

  // create pointcloud from voxel grid
  CreateEnvironmentVoxelGrid();

  // create pointcloud publisher and publish at constant frequency
  env_pc_pub_ = create_publisher<::sensor_msgs::msg::PointCloud2>(
      "~/" + env_pc_topic_, 10);
  env_pub_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&EnvironmentBuilder::TimerCallbackEnvironmentPC, this));

  // create voxel grid publisher and publish at constant frequency
  voxel_grid_pub_ = create_publisher<::env_builder_msgs::msg::VoxelGridStamped>(
      "~/" + env_vg_topic_, 10);
  voxel_grid_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&EnvironmentBuilder::TimerCallbackEnvironmentVG, this));

  // create service for drones to call and get the local voxel grid
  voxel_grid_service_ = create_service<::env_builder_msgs::srv::GetVoxelGrid>(
      "~/" + get_grid_service_name_,
      ::std::bind(&EnvironmentBuilder::GetVoxelGridService, this,
                  ::std::placeholders::_1, ::std::placeholders::_2));
}

::env_builder_msgs::msg::VoxelGridStamped
EnvironmentBuilder::GenerateVoxelGridMSG(::std::array<float, 3> &position,
                                         ::std::array<float, 3> &range) {

  ::env_builder_msgs::msg::VoxelGridStamped vg_msg;
  vg_msg.voxel_grid.voxel_size = vox_size_;

  // find the origin of the grid
  ::std::array<float, 3> origin;
  origin[0] = (position[0] - range[0] / 2);
  origin[1] = (position[1] - range[1] / 2);
  origin[2] = (position[2] - range[2] / 2);
  origin[0] = floor((origin[0] - origin_grid_[0]) / vox_size_) * vox_size_ +
              origin_grid_[0];
  origin[1] = floor((origin[1] - origin_grid_[1]) / vox_size_) * vox_size_ +
              origin_grid_[1];
  origin[2] = floor((origin[2] - origin_grid_[2]) / vox_size_) * vox_size_ +
              origin_grid_[2];

  vg_msg.voxel_grid.origin = origin;

  // find the range in integer dimensions
  ::std::array<uint32_t, 3> dim;
  dim[0] = floor(range[0] / vox_size_);
  dim[1] = floor(range[1] / vox_size_);
  dim[2] = floor(range[2] / vox_size_);

  vg_msg.voxel_grid.dimension = dim;

  // find the starting index
  ::std::vector<int> start_idx;
  start_idx.push_back(::std::round((origin[0] - origin_grid_[0]) / vox_size_));
  start_idx.push_back(::std::round((origin[1] - origin_grid_[1]) / vox_size_));
  start_idx.push_back(::std::round((origin[2] - origin_grid_[2]) / vox_size_));

  // generate the sub voxel grid from the environment
  vg_msg.voxel_grid.data.resize(dim[0] * dim[1] * dim[2]);
  for (int i = start_idx[0]; i < start_idx[0] + int(dim[0]); i++) {
    for (int j = start_idx[1]; j < start_idx[1] + int(dim[1]); j++) {
      for (int k = start_idx[2]; k < start_idx[2] + int(dim[2]); k++) {
        int i_msg = i - start_idx[0];
        int j_msg = j - start_idx[1];
        int k_msg = k - start_idx[2];
        int idx = i_msg + dim[0] * j_msg + dim[0] * dim[1] * k_msg;
        vg_msg.voxel_grid.data[idx] =
            voxel_grid_shared_ptr_->GetVoxelInt(::Eigen::Vector3i(i, j, k));
        if (free_grid_) {
          if (vg_msg.voxel_grid.data[idx] == -1) {
            vg_msg.voxel_grid.data[idx] = 0;
          }
        }
      }
    }
  }

  vg_msg.header.stamp = now();
  vg_msg.header.frame_id = env_pc_frame_;

  return vg_msg;
}

void EnvironmentBuilder::GetVoxelGridService(
    const ::std::shared_ptr<::env_builder_msgs::srv::GetVoxelGrid::Request>
        request,
    ::std::shared_ptr<::env_builder_msgs::srv::GetVoxelGrid::Response>
        response) {
  // process the request in a new thread to avoid a huge backlog in case
  // multiple agents are requesting information
  ::std::array<float, 3> position = request->position;
  ::std::array<float, 3> range = request->range;

  ::env_builder_msgs::msg::VoxelGridStamped vg_stamped_msg =
      GenerateVoxelGridMSG(position, range);
  response->voxel_grid_stamped = vg_stamped_msg;
}

void EnvironmentBuilder::TimerCallbackEnvironmentPC() {
  env_pc_msg_->header.stamp = now();
  env_pc_pub_->publish(*env_pc_msg_);
}

void EnvironmentBuilder::TimerCallbackEnvironmentVG() {
  voxel_grid_pub_->publish(voxel_grid_stamped_msg_);
}

void EnvironmentBuilder::CreateEnvironmentVoxelGrid() {

  // set voxel size
  voxel_grid_stamped_msg_.voxel_grid.voxel_size = vox_size_;

  // set origin of the grid
  voxel_grid_stamped_msg_.voxel_grid.origin[0] = origin_grid_[0];
  voxel_grid_stamped_msg_.voxel_grid.origin[1] = origin_grid_[1];
  voxel_grid_stamped_msg_.voxel_grid.origin[2] = origin_grid_[2];

  // set the integer dimensions of the grid
  voxel_grid_stamped_msg_.voxel_grid.dimension[0] =
      ceil(dimension_grid_[0] / vox_size_);
  voxel_grid_stamped_msg_.voxel_grid.dimension[1] =
      ceil(dimension_grid_[1] / vox_size_);
  voxel_grid_stamped_msg_.voxel_grid.dimension[2] =
      ceil(dimension_grid_[2] / vox_size_);

  // set the data of the grid
  voxel_grid_stamped_msg_.voxel_grid.data = voxel_grid_shared_ptr_->GetData();

  voxel_grid_stamped_msg_.header.stamp = now();
  voxel_grid_stamped_msg_.header.frame_id = env_pc_frame_;
}

void EnvironmentBuilder::CreateEnvironmentPointCloud() {
  ::Eigen::Vector3d origin_vg = voxel_grid_shared_ptr_->GetOrigin();
  ::Eigen::Vector3i dim_vg = voxel_grid_shared_ptr_->GetDim();
  double vox_size = voxel_grid_shared_ptr_->GetVoxSize();

  ::pcl::PointCloud<::pcl::PointXYZ> cloud_env;

  // add obstacles points to point cloud
  for (int i = 0; i < dim_vg(0); i++) {
    for (int j = 0; j < dim_vg(1); j++) {
      for (int k = 0; k < dim_vg(2); k++) {
        if (voxel_grid_shared_ptr_->IsOccupied(Eigen::Vector3i(i, j, k))) {
          ::pcl::PointXYZ pt;
          pt.x = i * vox_size + vox_size / 2 + origin_vg[0];
          pt.y = j * vox_size + vox_size / 2 + origin_vg[1];
          pt.z = k * vox_size + vox_size / 2 + origin_vg[2];
          cloud_env.points.push_back(pt);
        }
      }
    }
  }

  // create pc message
  env_pc_msg_ = ::std::make_shared<::sensor_msgs::msg::PointCloud2>();
  ::pcl::toROSMsg(cloud_env, *env_pc_msg_);
  env_pc_msg_->header.frame_id = env_pc_frame_;
}

void EnvironmentBuilder::AddObstacles() {
  srand(rand_seed_);

  int n_obst;
  if (multi_obst_position_) {
    n_obst = position_obst_vec_.size() / 3;
  } else {
    n_obst = n_obst_;
  }

  for (int i = 0; i < n_obst; i++) {

    ::Eigen::Vector3d center_obst;
    if (multi_obst_position_) {
      center_obst = ::Eigen::Vector3d(position_obst_vec_[3 * i],
                                      position_obst_vec_[3 * i + 1],
                                      position_obst_vec_[3 * i + 2]);
    } else {
      // generate the obstacles center we add 0.01 to avoid modulo by 0 which
      // is undefined behavior
      double eps = 0.02;
      center_obst(0) = ((rand() % int((range_obst_[0] + eps) * 100)) / 100) +
                       origin_obst_[0] - origin_grid_[0];
      center_obst(1) = ((rand() % int((range_obst_[1] + eps) * 100)) / 100) +
                       origin_obst_[1] - origin_grid_[1];
      center_obst(2) = ((rand() % int((range_obst_[2] + eps) * 100)) / 100) +
                       origin_obst_[2] - origin_grid_[2];
    }

    ::Eigen::Vector3d dim_obst;
    if (multi_obst_size_) {
      dim_obst =
          ::Eigen::Vector3d(size_obst_vec_[3 * i], size_obst_vec_[3 * i + 1],
                            size_obst_vec_[3 * i + 2]);
    } else {
      // generate the obstacle size
      dim_obst = ::Eigen::Vector3d(size_obst_[0], size_obst_[1], size_obst_[2]);
    }

    ::voxel_grid_util::AddObstacle(voxel_grid_shared_ptr_, center_obst,
                                   dim_obst);
  }
}

void EnvironmentBuilder::InflateObstacles() {
  voxel_grid_shared_ptr_->InflateObstacles(inflation_dist_);
}

void EnvironmentBuilder::CreatePotentialField() {
  voxel_grid_shared_ptr_->CreatePotentialField(potential_dist_, potential_pow_);
}

void EnvironmentBuilder::SaveObstacles() {
  // save pointcloud to csv file
  ::voxel_grid_util::WriteGridToFile(voxel_grid_shared_ptr_, "map.csv");
}

void EnvironmentBuilder::CreateEmptyVoxelGrid() {
  ::Eigen::Vector3d origin_tmp(origin_grid_.data());
  ::Eigen::Vector3d dimension_tmp(dimension_grid_.data());

  voxel_grid_shared_ptr_ = ::std::make_shared<::voxel_grid_util::VoxelGrid>(
      origin_tmp, dimension_tmp, vox_size_, free_grid_);
}

void EnvironmentBuilder::DeclareRosParameters() {
  declare_parameter("origin_grid", std::vector<double>(3, 0.0));
  declare_parameter("dimension_grid", std::vector<double>(3, 15.0));
  declare_parameter("vox_size", 0.3);
  declare_parameter("free_grid", true);
  declare_parameter("inflation_dist", 0.3);
  declare_parameter("potential_dist", 0.0);
  declare_parameter("potential_pow", 1.0);
  declare_parameter("save_obstacles", false);

  declare_parameter("multi_obst_size", false);
  declare_parameter("multi_obst_position", false);
  declare_parameter("range_obst", std::vector<double>(3, 12.0));
  declare_parameter("origin_obst", std::vector<double>(3, 2.0));
  declare_parameter("size_obst", std::vector<double>(3, 2.0));
  declare_parameter("size_obst_vec", std::vector<double>(3, 0.0));
  declare_parameter("position_obst_vec", std::vector<double>(3, 0.0));
  declare_parameter("n_obst", 10);
  declare_parameter("rand_seed", 1);

  declare_parameter("env_vg_topic", "environment_voxel_grid");
  declare_parameter("env_pc_topic", "environment_pointcloud");
  declare_parameter("env_pc_frame", "map");

  declare_parameter("get_grid_service_name", "get_voxel_grid");
}

void EnvironmentBuilder::InitializeRosParameters() {
  origin_grid_ = get_parameter("origin_grid").as_double_array();
  dimension_grid_ = get_parameter("dimension_grid").as_double_array();
  vox_size_ = get_parameter("vox_size").as_double();
  free_grid_ = get_parameter("free_grid").as_bool();
  inflation_dist_ = get_parameter("inflation_dist").as_double();
  potential_dist_ = get_parameter("potential_dist").as_double();
  potential_pow_ = get_parameter("potential_pow").as_double();
  save_obstacles_ = get_parameter("save_obstacles").as_bool();

  multi_obst_size_ = get_parameter("multi_obst_size").as_bool();
  multi_obst_position_ = get_parameter("multi_obst_position").as_bool();
  range_obst_ = get_parameter("range_obst").as_double_array();
  origin_obst_ = get_parameter("origin_obst").as_double_array();
  size_obst_ = get_parameter("size_obst").as_double_array();
  size_obst_vec_ = get_parameter("size_obst_vec").as_double_array();
  position_obst_vec_ = get_parameter("position_obst_vec").as_double_array();
  n_obst_ = get_parameter("n_obst").as_int();
  rand_seed_ = get_parameter("rand_seed").as_int();

  env_vg_topic_ = get_parameter("env_vg_topic").as_string();
  env_pc_topic_ = get_parameter("env_pc_topic").as_string();
  env_pc_frame_ = get_parameter("env_pc_frame").as_string();

  get_grid_service_name_ = get_parameter("get_grid_service_name").as_string();
}

} // namespace env_builder
