#include "map_builder.hpp"

namespace mapping_util {
MapBuilder::MapBuilder() : ::rclcpp::Node("map_builder") {
  // declare environment parameters
  DeclareRosParameters();

  // initialize parameters
  InitializeRosParameters();

  // set up a callback to execute code on shutdown
  on_shutdown(::std::bind(&MapBuilder::OnShutdown, this));

  // resize current position
  pos_curr_.resize(3);

  // set first transform received to false
  first_transform_received_ = false;

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

  // create frustum publisher
  ::std::string frustum_pub_topic = "agent_" + ::std::to_string(id_) + "/fov";
  frustum_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      frustum_pub_topic, 10);
}

void MapBuilder::DeclareRosParameters() {
  declare_parameter("env_vg_topic", "env_builder_node/environment_voxel_grid");
  declare_parameter("id", 0);
  declare_parameter("voxel_grid_range", ::std::vector<double>(3, 10.0));
  declare_parameter("world_frame", "world");
  declare_parameter("free_grid", true);
  declare_parameter("inflation_dist", 0.3);
  declare_parameter("potential_dist", 1.8);
  declare_parameter("potential_pow", 4.0);
  declare_parameter("fov_x", M_PI / 2);
  declare_parameter("fov_y", M_PI / 3);
  declare_parameter("fov_y_offset", 0.0);
  declare_parameter("frustum_length", 1.0);
  declare_parameter("limited_fov", true);
}

void MapBuilder::InitializeRosParameters() {
  env_vg_topic_ = get_parameter("env_vg_topic").as_string();
  id_ = get_parameter("id").as_int();
  voxel_grid_range_ = get_parameter("voxel_grid_range").as_double_array();
  world_frame_ = get_parameter("world_frame").as_string();
  free_grid_ = get_parameter("free_grid").as_bool();
  inflation_dist_ = get_parameter("inflation_dist").as_double();
  potential_dist_ = get_parameter("potential_dist").as_double();
  potential_pow_ = get_parameter("potential_pow").as_double();
  fov_x_ = get_parameter("fov_x").as_double();
  fov_y_ = get_parameter("fov_y").as_double();
  fov_y_offset_ = get_parameter("fov_y_offset").as_double();
  frustum_length_ = get_parameter("frustum_length").as_double();
  limited_fov_ = get_parameter("limited_fov").as_bool();
}

void MapBuilder::EnvironmentVoxelGridCallback(
    const ::env_builder_msgs::msg::VoxelGridStamped::SharedPtr vg_msg) {
  // first check if we received the first position
  if (first_transform_received_) {
    // start global timer
    auto t_start_wall_global = ::std::chrono::high_resolution_clock::now();

    // get voxel size
    double voxel_size = vg_msg->voxel_grid.voxel_size;

    // find the origin of the grid
    ::std::array<double, 3> origin_grid = vg_msg->voxel_grid.origin;
    ::Eigen::Vector3d origin;
    pos_mutex_.lock();
    ::Eigen::Vector3d pos_curr(pos_curr_[0], pos_curr_[1], pos_curr_[2]);
    origin[0] = (pos_curr_[0] - voxel_grid_range_[0] / 2);
    origin[1] = (pos_curr_[1] - voxel_grid_range_[1] / 2);
    origin[2] = (pos_curr_[2] - voxel_grid_range_[2] / 2);
    pos_mutex_.unlock();
    origin[0] = round((origin[0] - origin_grid[0]) / voxel_size) * voxel_size +
                origin_grid[0];
    origin[1] = round((origin[1] - origin_grid[1]) / voxel_size) * voxel_size +
                origin_grid[1];
    origin[2] = round((origin[2] - origin_grid[2]) / voxel_size) * voxel_size +
                origin_grid[2];

    // find the range in integer dimensions
    ::Eigen::Vector3i dim;
    dim[0] = floor(voxel_grid_range_[0] / voxel_size);
    dim[1] = floor(voxel_grid_range_[1] / voxel_size);
    dim[2] = floor(voxel_grid_range_[2] / voxel_size);

    // find the starting index
    ::std::vector<int> start_idx;
    start_idx.push_back(
        ::std::round((origin[0] - origin_grid[0]) / voxel_size));
    start_idx.push_back(
        ::std::round((origin[1] - origin_grid[1]) / voxel_size));
    start_idx.push_back(
        ::std::round((origin[2] - origin_grid[2]) / voxel_size));

    // generate the sub voxel grid from the environment
    ::voxel_grid_util::VoxelGrid vg(origin, dim, voxel_size, free_grid_);
    ::std::array<uint32_t, 3> dim_env = vg_msg->voxel_grid.dimension;

    for (int i = start_idx[0]; i < start_idx[0] + int(dim[0]); i++) {
      for (int j = start_idx[1]; j < start_idx[1] + int(dim[1]); j++) {
        for (int k = start_idx[2]; k < start_idx[2] + int(dim[2]); k++) {
          int i_msg = i - start_idx[0];
          int j_msg = j - start_idx[1];
          int k_msg = k - start_idx[2];
          int idx_env =
              i + int(dim_env[0]) * j + int(dim_env[0]) * int(dim_env[1]) * k;
          int8_t data_val;
          if (i < 0 || j < 0 || k < 0 || i >= dim_env[0] || j >= dim_env[1] ||
              k >= dim_env[2]) {
            data_val = -1;
          } else {
            data_val = vg_msg->voxel_grid.data[idx_env];
          }
          vg.SetVoxelInt(::Eigen::Vector3i(i_msg, j_msg, k_msg), data_val);
          if (free_grid_) {
            if (data_val == -1) {
              vg.SetVoxelInt(::Eigen::Vector3i(i_msg, j_msg, k_msg), 0);
            }
          } else {
            if (data_val == 0) {
              vg.SetVoxelInt(::Eigen::Vector3i(i_msg, j_msg, k_msg), -1);
            }
          }
        }
      }
    }

    // if we do not wish to set all voxels to free, it means they are unknown
    // and we need to raycast in visible field of the agent to free them and
    // then merge the current grid with the newly raycast grid;
    if (!free_grid_) {
      // if this is the first iteration, set the current voxel grid to the voxel
      // grid we just received but clear the voxels around the agent
      if (voxel_grid_curr_.GetData().size() == 0) {
        ::Eigen::Vector3d origin = vg.GetOrigin();
        ::Eigen::Vector3i dim = vg.GetDim();
        voxel_grid_curr_ =
            ::voxel_grid_util::VoxelGrid(origin, dim, vg.GetVoxSize(), false);
        // clear voxels around the center
        ClearVoxelsCenter();
      }
      // first raycast from the center of the grid to clear voxels
      ::Eigen::Vector3d pos_curr_local = vg.GetCoordLocal(pos_curr);
      auto t_start_wall = ::std::chrono::high_resolution_clock::now();
      RaycastAndClear(vg, pos_curr_local);
      auto t_end_wall = ::std::chrono::high_resolution_clock::now();
      double raycast_time_wall_ms =
          ::std::chrono::duration_cast<::std::chrono::nanoseconds>(t_end_wall -
                                                                   t_start_wall)
              .count();
      // convert from nano to milliseconds
      raycast_time_wall_ms *= 1e-6;
      // save wall computation time
      raycast_comp_time_.push_back(raycast_time_wall_ms);

      // then merge the voxel grid and set voxel_grid_curr_ to the new merged
      // grid
      t_start_wall = ::std::chrono::high_resolution_clock::now();

      voxel_grid_curr_ = MergeVoxelGrids(voxel_grid_curr_, vg);

      t_end_wall = ::std::chrono::high_resolution_clock::now();
      double merging_time_wall_ms =
          ::std::chrono::duration_cast<::std::chrono::nanoseconds>(t_end_wall -
                                                                   t_start_wall)
              .count();
      // convert from nano to milliseconds
      merging_time_wall_ms *= 1e-6;
      // save wall computation time
      merge_comp_time_.push_back(merging_time_wall_ms);

    } else {
      // if we don't need to raycast, then the voxel_grid that we save is the
      // same as the one that we receive from the environment
      voxel_grid_curr_ = vg;
    }

    // voxel grid to publish
    ::voxel_grid_util::VoxelGrid voxel_grid = voxel_grid_curr_;

    // inflate the unknown voxels by the inflation distance to guarantee safety
    // when computing the safe corridor;
    SetUncertainToUnknown(voxel_grid);

    // inflate obstacles
    voxel_grid.InflateObstacles(inflation_dist_);

    // create potential field in only free voxels (unknown voxels are kept
    // unknown)
    voxel_grid.CreatePotentialField(potential_dist_, potential_pow_);

    // create the final grid message and publish it
    ::env_builder_msgs::msg::VoxelGrid vg_final_msg =
        ConvertVGUtilToVGMsg(voxel_grid);

    ::env_builder_msgs::msg::VoxelGridStamped vg_final_msg_stamped;
    vg_final_msg_stamped.voxel_grid = vg_final_msg;
    vg_final_msg_stamped.voxel_grid.voxel_size = voxel_size;
    vg_final_msg_stamped.header.stamp = now();
    vg_final_msg_stamped.header.frame_id = world_frame_;

    auto t_end_wall_global = ::std::chrono::high_resolution_clock::now();
    double tot_time_wall_ms =
        ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
            t_end_wall_global - t_start_wall_global)
            .count();
    // convert from nano to milliseconds
    tot_time_wall_ms *= 1e-6;
    // save wall computation time
    tot_comp_time_.push_back(tot_time_wall_ms);

    voxel_grid_pub_->publish(vg_final_msg_stamped);
  }
}

::voxel_grid_util::VoxelGrid
MapBuilder::MergeVoxelGrids(const ::voxel_grid_util::VoxelGrid &vg_old,
                            const ::voxel_grid_util::VoxelGrid &vg_new) {
  // create final voxel grid
  ::voxel_grid_util::VoxelGrid vg_final = vg_new;

  // update the final voxel grid by going through each voxel and merging the old
  // with the new
  double voxel_size = vg_final.GetVoxSize();
  ::Eigen::Vector3i dim = vg_final.GetDim();
  ::Eigen::Vector3d offset_double = (vg_final.GetOrigin() - vg_old.GetOrigin());
  /* ::std::cout << "vg_final origin:" << vg_final.GetOrigin().transpose() */
              /* << ::std::endl; */
  /* ::std::cout << "vg_old origin:" << vg_old.GetOrigin().transpose() */
              /* << ::std::endl; */
  ::Eigen::Vector3i offset_int;
  offset_int[0] = round(offset_double[0] / voxel_size);
  offset_int[1] = round(offset_double[1] / voxel_size);
  offset_int[2] = round(offset_double[2] / voxel_size);
  /* ::std::cout << "offset_int: " << offset_int.transpose() << ::std::endl; */
  for (int i = 0; i < dim[0]; i++) {
    for (int j = 0; j < dim[1]; j++) {
      for (int k = 0; k < dim[2]; k++) {
        // the voxels of the new voxel grid stay the same unless they are
        // unknown; in that cast we replace them with the values seen in the old
        // voxel grid
        ::Eigen::Vector3i coord(i, j, k);
        if (vg_final.IsUnknown(coord)) {
          ::Eigen::Vector3i coord_final = coord + offset_int;
          int8_t vox_value = vg_old.GetVoxelInt(coord_final);
          vg_final.SetVoxelInt(coord, vox_value);
        }
      }
    }
  }
  return vg_final;
}

void MapBuilder::RaycastAndClear(::voxel_grid_util::VoxelGrid &vg,
                                 const ::Eigen::Vector3d &start) {
  /* this implementation is for 360 degrees raycasting i.e. we assume we see 360
   degrees around the drone with cameras; the idea is to raycast to the border
   voxels of the grid; this way we guarantee that we don't miss a voxel while
   maintaining the number of raycasted lines low */

  // get params
  ::Eigen::Vector3d origin = vg.GetOrigin();
  ::Eigen::Vector3i dim = vg.GetDim();

  // create final voxel grid
  ::voxel_grid_util::VoxelGrid vg_final(origin, dim, vg.GetVoxSize(), false);

  // first raycast the ceiling and the floor
  ::std::vector<int> k_vec = {0, dim(2) - 1};
  for (int i = 0; i < dim(0); i++) {
    for (int j = 0; j < dim(1); j++) {
      for (int k : k_vec) {
        ::Eigen::Vector3d end(i + 0.5, j + 0.5, k + 0.5);
        ClearLine(vg, vg_final, start, end);
      }
    }
  }

  // then raycast the wall with fixed y coordinate
  ::std::vector<int> j_vec = {0, dim(1) - 1};
  for (int i = 0; i < dim(0); i++) {
    for (int k = 0; k < dim(2); k++) {
      for (int j : j_vec) {
        ::Eigen::Vector3d end(i + 0.5, j + 0.5, k + 0.5);
        ClearLine(vg, vg_final, start, end);
      }
    }
  }

  // then raycast the wall with fixed x coordinate
  ::std::vector<int> i_vec = {0, dim(0) - 1};
  for (int j = 0; j < dim(1); j++) {
    for (int k = 0; k < dim(2); k++) {
      for (int i : i_vec) {
        ::Eigen::Vector3d end(i + 0.5, j + 0.5, k + 0.5);
        ClearLine(vg, vg_final, start, end);
      }
    }
  }

  // set vg to vg_final
  vg = vg_final;
}

void MapBuilder::SetUncertainToUnknown(::voxel_grid_util::VoxelGrid &vg) {
  // create final grid
  ::voxel_grid_util::VoxelGrid vg_final = vg;

  // first get the cube size around the unknonw voxel to set as unknown
  int cube_size = ceil(inflation_dist_ / vg.GetVoxSize());

  // get the params
  ::Eigen::Vector3i dim = vg.GetDim();

  // go through every point in the voxel grid and if its unknown, set all the
  // voxels around it to unknown unless they are occupied
  for (int i = cube_size; i < dim[0] - cube_size; i++) {
    for (int j = cube_size; j < dim[1] - cube_size; j++) {
      for (int k = cube_size; k < dim[2] - cube_size; k++) {
        ::Eigen::Vector3i pt(i, j, k);
        if (vg.IsUnknown(pt)) {
          for (int i_new = -cube_size; i_new <= cube_size; i_new++) {
            for (int j_new = -cube_size; j_new <= cube_size; j_new++) {
              for (int k_new = -cube_size; k_new <= cube_size; k_new++) {
                ::Eigen::Vector3i neighbour(i + i_new, j + j_new, k + k_new);
                if (!vg.IsOccupied(neighbour)) {
                  vg_final.SetVoxelInt(neighbour, ENV_BUILDER_UNK);
                }
              }
            }
          }
        }
      }
    }
  }

  // set the voxel grid to the final voxel grid
  vg = vg_final;
}

void MapBuilder::ClearLine(::voxel_grid_util::VoxelGrid &vg,
                           ::voxel_grid_util::VoxelGrid &vg_final,
                           const ::Eigen::Vector3d &start,
                           const ::Eigen::Vector3d &end) {
  // only clear the line if is it within the field of view
  bool in_fov = false;

  if (limited_fov_) {
    // get x_b, y_b, and z_b from quaternion
    ::Eigen::Vector3d x_b = rot_mat_cam_.col(0);
    ::Eigen::Vector3d y_b = rot_mat_cam_.col(1);
    ::Eigen::Vector3d z_b = rot_mat_cam_.col(2);

    // project the raycasting vector on the x-z plane (we are using x forward, y
    // left and z up, so fov_x of the camera convention is really in the y
    // direction and fov_y is in the z direction).
    ::Eigen::Vector3d dir = end - start;
    ::Eigen::Vector3d dir_xz = dir - dir.dot(y_b) * y_b;
    dir_xz.normalize();
    ::Eigen::Vector3d dir_xy = dir - dir.dot(z_b) * y_b;
    dir_xz.normalize();
    // if the line is in the furstum/fov than set in_fov to true
    if (dir_xz.dot(x_b) > cos(fov_y_ / 2) &&
        dir_xy.dot(x_b) > cos(fov_x_ / 2)) {
      in_fov = true;
    }
  } else {
    in_fov = true;
  }

  if (in_fov) {
    ::Eigen::Vector3d collision_pt;
    ::std::vector<::Eigen::Vector3d> visited_points;
    double max_dist_raycast = (start - end).norm();
    bool line_clear = ::path_finding_util::IsLineClear(
        start, end, vg, max_dist_raycast, collision_pt, visited_points);
    // if line is not clear than the last point is a collision point and we
    // don't need to clear it in the voxel grid
    if (line_clear) {
      visited_points.push_back(end);
    } else {
      ::Eigen::Vector3d last_point = (end - start) * 1e-7 + collision_pt;
      ::Eigen::Vector3i last_point_int(last_point[0], last_point[1],
                                       last_point[2]);
      // check around last_point_int to see the voxels that are occupied;
      vg_final.SetVoxelInt(last_point_int, ENV_BUILDER_OCC);
      /* for (int i = -1; i <= 1; i++) { */
      /*   for (int j = -1; j <= 1; j++) { */
      /*     for (int k = -1; k <= 1; k++) { */
      /*       ::Eigen::Vector3i new_pt = */
      /*           last_point_int + ::Eigen::Vector3i(i, j, k); */
      /*       if (vg.IsOccupied(new_pt)) { */
      /*         vg_final.SetVoxelInt(new_pt, ENV_BUILDER_OCC); */
      /*       } */
      /*     } */
      /*   } */
      /* } */
    }

    int vec_size = visited_points.size();
    for (int i = 0; i < vec_size - 1; i++) {
      vg_final.SetVoxelInt(
          ::Eigen::Vector3i(
              (visited_points[i](0) + visited_points[i + 1](0)) / 2.0,
              (visited_points[i](1) + visited_points[i + 1](1)) / 2.0,
              (visited_points[i](2) + visited_points[i + 1](2)) / 2.0),
          0);
    }
  }
}

void MapBuilder::ClearVoxelsCenter() {
  ::Eigen::Vector3d pos_curr(pos_curr_[0], pos_curr_[1], pos_curr_[2]);
  ::Eigen::Vector3d pos_curr_local = voxel_grid_curr_.GetCoordLocal(pos_curr);
  int i_mid = floor(pos_curr_local(0));
  int j_mid = floor(pos_curr_local(1));
  int k_mid = floor(pos_curr_local(2));
  for (int i = i_mid - 2; i <= i_mid + 2; i++) {
    for (int j = j_mid - 2; j <= j_mid + 2; j++) {
      for (int k = k_mid - 2; k <= k_mid + 2; k++) {
        voxel_grid_curr_.SetVoxelInt(::Eigen::Vector3i(i, j, k), 0);
      }
    }
  }
}

void MapBuilder::TfCallback(const ::tf2_msgs::msg::TFMessage::SharedPtr msg) {
  for (const auto &transform_stamped : msg->transforms) {
    if (transform_stamped.header.frame_id == world_frame_ &&
        transform_stamped.child_frame_id == agent_frame_) {
      // get the position from the transform
      const ::geometry_msgs::msg::Transform &transform =
          transform_stamped.transform;
      pos_curr_[0] = transform.translation.x;
      pos_curr_[1] = transform.translation.y;
      pos_curr_[2] = transform.translation.z;

      // get attitude in quaternion
      ::Eigen::Quaterniond quat_curr;
      quat_curr.x() = transform.rotation.x;
      quat_curr.y() = transform.rotation.y;
      quat_curr.z() = transform.rotation.z;
      quat_curr.w() = transform.rotation.w;
      // get x_b, y_b, and z_b from quaternion
      ::Eigen::Matrix3d rot_mat = quat_curr.normalized().toRotationMatrix();
      ::Eigen::Vector3d x_b = rot_mat.col(0);
      ::Eigen::Vector3d y_b = rot_mat.col(1);
      ::Eigen::Vector3d z_b = rot_mat.col(2);
      // compute x_b_rotated from the tilt of the camera (fov_y_offset_)
      ::Eigen::Vector3d x_b_rotated =
          cos(fov_y_offset_) * x_b + sin(fov_y_offset_) * z_b;
      ::Eigen::Vector3d z_b_rotated =
          cos(fov_y_offset_) * z_b - sin(fov_y_offset_) * x_b;
      // set the camera rotation matrix
      rot_mat_cam_.col(0) = x_b_rotated;
      rot_mat_cam_.col(1) = y_b;
      rot_mat_cam_.col(2) = z_b_rotated;

      // set first_transform_received_ to true
      first_transform_received_ = true;

      // publish camera frustum
      if (limited_fov_) {
        PublishFrustum(transform_stamped);
      }
    }
  }
}

void MapBuilder::PublishFrustum(
    const ::geometry_msgs::msg::TransformStamped &tf_stamped) {
  // build marker array from lines of the frustum
  ::visualization_msgs::msg::MarkerArray marker_array;

  // define the points of the frustum
  ::geometry_msgs::msg::Point point_0, point_1, point_2, point_3, point_4;
  point_0.x = 0;
  point_0.y = 0;
  point_0.z = 0;

  // compute rotated x_b and rotated z_b in the body frame
  ::Eigen::Vector3d x_b(1, 0, 0);
  ::Eigen::Vector3d y_b(0, 1, 0);
  ::Eigen::Vector3d z_b(0, 0, 1);
  ::Eigen::Vector3d x_b_rotated =
      cos(fov_y_offset_) * x_b + sin(fov_y_offset_) * z_b;
  ::Eigen::Vector3d z_b_rotated =
      cos(fov_y_offset_) * z_b - sin(fov_y_offset_) * x_b;
  ::Eigen::Vector3d top_left = frustum_length_ * x_b_rotated +
                               tan(fov_x_ / 2) * frustum_length_ * y_b +
                               tan(fov_y_ / 2) * frustum_length_ * z_b;
  ::Eigen::Vector3d bottom_left = frustum_length_ * x_b_rotated +
                                  tan(fov_x_ / 2) * frustum_length_ * y_b -
                                  tan(fov_y_ / 2) * frustum_length_ * z_b;
  ::Eigen::Vector3d top_right = frustum_length_ * x_b_rotated -
                                tan(fov_x_ / 2) * frustum_length_ * y_b +
                                tan(fov_y_ / 2) * frustum_length_ * z_b;
  ::Eigen::Vector3d bottom_right = frustum_length_ * x_b_rotated -
                                   tan(fov_x_ / 2) * frustum_length_ * y_b -
                                   tan(fov_y_ / 2) * frustum_length_ * z_b;

  // set the base of the frustum
  point_1.x = top_left(0);
  point_1.y = top_left(1);
  point_1.z = top_left(2);

  point_2.x = bottom_left(0);
  point_2.y = bottom_left(1);
  point_2.z = bottom_left(2);

  point_3.x = top_right(0);
  point_3.y = top_right(1);
  point_3.z = top_right(2);

  point_4.x = bottom_right(0);
  point_4.y = bottom_right(1);
  point_4.z = bottom_right(2);

  // create the lines
  ::visualization_msgs::msg::Marker line_0, line_1, line_2, line_3, line_4,
      line_5, line_6, line_7;
  line_0.header.frame_id = agent_frame_;
  line_0.header.stamp = tf_stamped.header.stamp;
  line_0.id = 0;
  line_0.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_0.action = visualization_msgs::msg::Marker::ADD;
  line_0.scale.x = 0.1; // Line width
  line_0.color.b = 1.0; // Line color (red)
  line_0.color.a = 1.0; // Line transparency

  line_1 = line_0;
  line_2 = line_0;
  line_3 = line_0;
  line_4 = line_0;
  line_5 = line_0;
  line_6 = line_0;
  line_7 = line_0;

  line_1.id = 1;
  line_2.id = 2;
  line_3.id = 3;
  line_4.id = 4;
  line_5.id = 5;
  line_6.id = 6;
  line_7.id = 7;

  line_0.points.push_back(point_0);
  line_0.points.push_back(point_1);

  line_1.points.push_back(point_0);
  line_1.points.push_back(point_2);

  line_2.points.push_back(point_0);
  line_2.points.push_back(point_3);

  line_3.points.push_back(point_0);
  line_3.points.push_back(point_4);

  line_4.points.push_back(point_1);
  line_4.points.push_back(point_2);

  line_5.points.push_back(point_2);
  line_5.points.push_back(point_4);

  line_6.points.push_back(point_4);
  line_6.points.push_back(point_3);

  line_7.points.push_back(point_3);
  line_7.points.push_back(point_1);

  // add the lines to the marker_array
  marker_array.markers.push_back(line_0);
  marker_array.markers.push_back(line_1);
  marker_array.markers.push_back(line_2);
  marker_array.markers.push_back(line_3);
  marker_array.markers.push_back(line_4);
  marker_array.markers.push_back(line_5);
  marker_array.markers.push_back(line_6);
  marker_array.markers.push_back(line_7);

  // publish the frustum
  frustum_pub_->publish(marker_array);
}

void MapBuilder::DisplayCompTime(::std::vector<double> &comp_time) {
  double max_t = 0;
  double min_t = 1e10;
  double sum_t = 0;
  double std_dev_t = 0;
  for (int i = 0; i < int(comp_time.size()); i++) {
    if (comp_time[i] > max_t) {
      max_t = comp_time[i];
    }
    if (comp_time[i] < min_t) {
      min_t = comp_time[i];
    }
    sum_t = sum_t + comp_time[i];
  }
  double mean_t = sum_t / comp_time.size();

  for (int i = 0; i < int(comp_time.size()); i++) {
    std_dev_t += (comp_time[i] - mean_t) * (comp_time[i] - mean_t);
  }
  std_dev_t = std_dev_t / comp_time.size();
  std_dev_t = sqrt(std_dev_t);

  ::std::cout << ::std::endl << "mean: " << mean_t;
  ::std::cout << ::std::endl << "std_dev: " << std_dev_t;
  ::std::cout << ::std::endl << "max: " << max_t;
  ::std::cout << ::std::endl << "min: " << min_t << ::std::endl;
}

void MapBuilder::OnShutdown() {
  ::std::cout << ::std::endl << "raycast: ";
  DisplayCompTime(raycast_comp_time_);
  ::std::cout << ::std::endl << "merge: ";
  DisplayCompTime(merge_comp_time_);
  ::std::cout << ::std::endl << "total: ";
  DisplayCompTime(tot_comp_time_);
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
