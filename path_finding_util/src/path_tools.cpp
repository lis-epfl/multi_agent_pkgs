#include "path_tools.hpp"

namespace path_finding_util {

::std::vector<::std::vector<double>>
EigenToVector(::std::vector<::Eigen::Vector3d> path) {

  ::std::vector<::std::vector<double>> path_out;
  for (auto pt : path) {
    ::std::vector<double> pt_vec = {pt[0], pt[1], pt[2]};
    path_out.push_back(pt_vec);
  }
  return path_out;
}

::std::vector<::Eigen::Vector3d>
VectorToEigen(::std::vector<::std::vector<double>> path) {
  ::std::vector<::Eigen::Vector3d> path_out;
  for (auto pt : path) {
    path_out.push_back(::Eigen::Vector3d(pt[0], pt[1], pt[2]));
  }
  return path_out;
}

double GetPathLength(const std::vector<Eigen::Vector3d> &path) {
  double total_length = 0;
  for (int i = 0; i < path.size() - 1; i++) {
    double segment_length = (path[i + 1] - path[i]).norm();
    total_length = total_length + segment_length;
  }
  return total_length;
}

std::vector<Eigen::Vector3d>
PathGlobalToLocal(const std::vector<Eigen::Vector3d> &path,
                  const ::voxel_grid_util::VoxelGrid &vg) {
  std::vector<Eigen::Vector3d> path_local;
  for (int i = 0; i < path.size(); i++) {
    path_local.push_back(vg.GetCoordLocal(path[i]));
  }

  return path_local;
}

std::vector<Eigen::Vector3d>
PathLocalToGlobal(const std::vector<Eigen::Vector3d> &path,
                  const ::voxel_grid_util::VoxelGrid &vg) {

  std::vector<Eigen::Vector3d> path_global;
  for (int i = 0; i < path.size(); i++) {
    path_global.push_back(vg.GetCoordGlobal(path[i]));
  }

  return path_global;
}

std::vector<Eigen::Vector3d>
DiscretizePathStep(const std::vector<Eigen::Vector3d> &path,
                   const double dis_step, const bool keep_orig) {
  std::vector<Eigen::Vector3d> path_dis;
  path_dis.push_back(path.front());
  Eigen::Vector3d ref_pt = path[1];
  Eigen::Vector3d curr_point = path[0];
  int path_idx = 1;
  while (!curr_point.isApprox(path.back(), PATH_FINDING_UTIL_COMP_PREC)) {
    Eigen::Vector3d mov_dir = ref_pt - curr_point;
    // if the distance bewteen the current point and the next path point is
    // smaller than dis_step
    if (mov_dir.norm() >= dis_step) {
      curr_point = curr_point + mov_dir.normalized() * dis_step;
      path_dis.push_back(curr_point);
    } else {
      double dis_step_tmp = dis_step;
      bool point_found = false;
      while (!point_found) {
        // first check if its the last point in the path
        if (path_idx == path.size() - 1) {
          curr_point = path.back();
          path_dis.push_back(curr_point);
          point_found = true;
        } else {
          // save ref_pt if we want to keep nodes of original path
          if (keep_orig) {
            path_dis.push_back(ref_pt);
          }
          // find the point along the path by walking dis_step
          curr_point = ref_pt;
          path_idx++;
          ref_pt = path[path_idx];
          dis_step_tmp -= mov_dir.norm();
          mov_dir = ref_pt - curr_point;
          if (mov_dir.norm() < dis_step_tmp) {
            continue;
          } else {
            curr_point = curr_point + mov_dir.normalized() * dis_step_tmp;
            path_dis.push_back(curr_point);
            point_found = true;
          }
        }
      }
    }
  }
  return path_dis;
}

std::vector<Eigen::Vector3d>
DiscretizePathPoints(const std::vector<Eigen::Vector3d> &path,
                     const int n_points, const bool keep_orig) {
  // first compute the step to discretize at
  double total_length = GetPathLength(path);

  // discretize the path according to this step
  double dis_step = total_length / (n_points + 1);
  return DiscretizePathStep(path, dis_step, keep_orig);
}

bool IsLineClear(const Eigen::Vector3d &pt_start, const Eigen::Vector3d &pt_end,
                 const ::voxel_grid_util::VoxelGrid &vg,
                 const double max_dist_raycast, const bool verbose) {

  // first check if both the start and the end points are inside the grid
  Eigen::Vector3i pt_start_i(pt_start(0), pt_start(1), pt_start(2));
  Eigen::Vector3i pt_end_i(pt_end(0), pt_end(1), pt_end(2));
  if (!vg.IsInsideGridInt(pt_start_i)) {
    std::cout << "Warning: start of the ray is outside the grid"
              << pt_start_i.transpose() << std::endl;
  }

  if (!vg.IsInsideGridInt(pt_end_i)) {
    std::cout << "Warning: end of the ray is outside the grid"
              << pt_end_i.transpose() << std::endl;
  }

  // Raycast and check if the final position is the same as the collision_pt
  Eigen::Vector3d collision_pt(-1, -1, -1);
  std::vector<Eigen::Vector3d> output = ::voxel_grid_util::Raycast(
      pt_start, pt_end, collision_pt, vg, max_dist_raycast, verbose);
  if (verbose) {
    WritePathToFile(output, "path_test.csv");
  }
  if (collision_pt(0) == -1) {
    return true;
  } else {
    return false;
  }
}

bool IsLineClear(const Eigen::Vector3d &pt_start, const Eigen::Vector3d &pt_end,
                 const ::voxel_grid_util::VoxelGrid &vg,
                 const double max_dist_raycast, Eigen::Vector3d &collision_pt,
                 std::vector<Eigen::Vector3d> &visited_points,
                 const bool verbose) {

  // first check if both the start and the end points are inside the grid
  Eigen::Vector3i pt_start_i(pt_start(0), pt_start(1), pt_start(2));
  Eigen::Vector3i pt_end_i(pt_end(0), pt_end(1), pt_end(2));
  if (!vg.IsInsideGridInt(pt_start_i)) {
    std::cout << "Warning: start of the ray is outside the grid "
              << pt_start_i.transpose() << std::endl;
  }

  if (!vg.IsInsideGridInt(pt_end_i)) {
    std::cout << "Warning: end of the ray is outside the grid "
              << pt_end_i.transpose() << std::endl;
  }

  // Raycast and check if the final position is the same as the collision_pt
  Eigen::Vector3d col_pt(-1, -1, -1);
  visited_points = ::voxel_grid_util::Raycast(pt_start, pt_end, col_pt, vg,
                                              max_dist_raycast, verbose);
  if (verbose) {
    WritePathToFile(visited_points, "path_test.csv");
  }
  if (col_pt(0) == -1) {
    return true;
  } else {
    collision_pt = col_pt;
    return false;
  }
}

std::vector<Eigen::Vector3d>
ShortenPath(const std::vector<Eigen::Vector3d> &path,
            const ::voxel_grid_util::VoxelGrid &vg, const double dis_step,
            const double max_dist_raycast, const bool verbose) {

  // declare final path
  std::vector<Eigen::Vector3d> path_final;
  path_final.push_back(path.front());

  // first discretize the path from the second node
  std::vector<Eigen::Vector3d> path_2 = path;
  path_2.erase(path_2.begin());
  std::vector<Eigen::Vector3d> path_dis =
      DiscretizePathStep(path_2, dis_step, true);

  // find the intermediate points
  // In order to find them, we must keep in memory the previously raycasted
  // points. Then, we find among them the point flip_pt where the dot product
  // between the direction vector of the raycast and (collision_pt - flip_pt)
  // flips sign (from + to -). This point will be added to the path.
  Eigen::Vector3d start_point = path_final.back();
  std::vector<Eigen::Vector3d> visited_points;
  std::vector<Eigen::Vector3d> visited_points_previous;
  Eigen::Vector3d collision_pt;

  // increase iteration counter
  // first find breaking point
  int offset_dis = 1;
  for (int i = 0; i < path_dis.size(); i++) {
    /* std::cout << "start point: " << start_point.transpose() << std::endl; */
    /* std::cout << "end point: " << path_dis[i].transpose() << std::endl; */
    if (IsLineClear(start_point, path_dis[i], vg, max_dist_raycast,
                    collision_pt, visited_points, verbose)) {
      /* std::cout << "line is clear" << std::endl; */
      // If the line is clear, save the output
      visited_points_previous = visited_points;
      // update reached_end
      if (i == path_dis.size() - 1) {
        path_final.push_back(path_dis[i]);
      }
    } else {
      i = std::max(i - 1, 0);
      path_final.push_back(path_dis[i]);
      start_point = path_dis[i];
      /* // There is a collision. Check for sign flip of the product */
      /* Eigen::Vector3d dir = (visited_points_previous.back() - start_point);
       */
      /* for (int j = 0; j < visited_points_previous.size(); j++) { */
      /*   Eigen::Vector3d dir_col = (collision_pt -
       * visited_points_previous[j]); */
      /*   // if the dot product became negative or we reached the last point */
      /*   // then save the point to the final path and update start point */
      /*   if (dir.dot(dir_col) < 0 || j == visited_points_previous.size() - 1)
       * { */
      /*     path_final.push_back(visited_points_previous[std::min( */
      /*         (int)(visited_points_previous.size() - 1), j + offset_dis)]);
       */
      /*     start_point = path_final.back(); */
      /*     break; */
      /*   } */
      /* } */
      // update the iterator i
    }
  }
  return path_final;
}

::std::vector<::std::vector<double>>
ShortenDMPPath(::std::vector<::std::vector<double>> &path,
               ::voxel_grid_util::VoxelGrid &vg) {
  // convert the path to eigen
  ::std::vector<::Eigen::Vector3d> path_eig =
      ::path_finding_util::VectorToEigen(path);

  // convert the path from global to local
  ::std::vector<::Eigen::Vector3d> path_eig_local =
      ::path_finding_util::PathGlobalToLocal(path_eig, vg);

  // start from the first point of the path and walk until we reach a point that
  // is not occupied or in a potential field
  int i = 0;
  while (i < int(path_eig_local.size()) - 1) {
    if (int(vg.GetVoxelInt(path_eig_local[i])) <= 0) {
      int i_start = i;
      int i_end = i;
      int j = i + 1;
      while (j < int(path_eig_local.size())) {
        bool line_clear = true;
        ::std::vector<Eigen::Vector3d> visited_points;
        ::Eigen::Vector3d collision_pt;
        ::Eigen::Vector3d start = path_eig_local[i_start];
        ::Eigen::Vector3d end = path_eig_local[j];
        double dist_start_end = (start - end).norm();
        bool collision = !::path_finding_util::IsLineClear(
            start, end, vg, dist_start_end, collision_pt, visited_points);
        if (collision) {
          line_clear = false;
        } else {
          for (auto &pt : visited_points) {
            if (int(vg.GetVoxelInt(pt)) > 0) {
              line_clear = false;
            }
          }
        }
        // check if line clear and voxel is clear
        if (int(vg.GetVoxelInt(path_eig_local[j])) <= 0 && line_clear) {
          i_end = j;
          j = j + 1;
        } else {
          break;
        }
      }
      // remove the points between i_start and i_end
      if (i_end > i_start) {
        path_eig_local.erase(path_eig_local.begin() + i_start + 1,
                             path_eig_local.begin() + i_end);
      }
    }
    i = i + 1;
  }

  // convert the path from local to global
  ::std::vector<::Eigen::Vector3d> path_eig_global =
      ::path_finding_util::PathLocalToGlobal(path_eig_local, vg);

  // convert eigen path to vector path
  ::std::vector<::std::vector<double>> path_out =
      ::path_finding_util::EigenToVector(path_eig_global);

  return path_out;
}

bool IsHomotopy(const std::vector<Eigen::Vector3d> &path_1,
                const std::vector<Eigen::Vector3d> &path_2,
                const ::voxel_grid_util::VoxelGrid &vg, const int n_points,
                const double max_dist_raycast, const bool verbose) {
  std::vector<Eigen::Vector3d> path_1_dis;
  std::vector<Eigen::Vector3d> path_2_dis;
  int n_points_final;
  if (n_points > 0) {
    n_points_final = n_points;
  } else {
    // find the number of points required so that the distance beween 2
    // discrete/sampled points is smaller than the voxel size
    double l_1 = GetPathLength(path_1);
    double l_2 = GetPathLength(path_2);
    double max_length_path = std::max(l_1, l_2);
    n_points_final = std::ceil(max_length_path);
  }

  path_1_dis = DiscretizePathPoints(path_1, n_points_final);
  path_2_dis = DiscretizePathPoints(path_2, n_points_final);

  Eigen::Vector3d collision_pt;
  // usually no need to check the first and last point, but in case we use this
  // function for other purposes
  for (int i = 0; i < n_points_final + 2; i++) {
    if (!IsLineClear(path_1_dis[i], path_2_dis[i], vg, max_dist_raycast)) {
      if (verbose) {
        std::cout << "line not clear: " << path_1_dis[i].transpose() << " "
                  << path_2_dis[i].transpose() << std::endl;
      }
      return false;
    }
  }
  return true;
}

std::vector<std::vector<Eigen::Vector3d>>
PruneEquivalentPaths(const std::vector<std::vector<Eigen::Vector3d>> &paths,
                     const ::voxel_grid_util::VoxelGrid &vg,
                     const double max_dist_raycast) {
  std::vector<std::vector<Eigen::Vector3d>> pruned_paths;
  if (paths.size() < 1)
    return pruned_paths;

  /* ---------- prune topo equivalent path ---------- */
  // output: pruned_paths
  std::vector<int> existing_paths_id;
  existing_paths_id.push_back(0);

  for (int i = 1; i < paths.size(); ++i) {
    // compare with existing paths
    bool new_path = true;

    for (int j = 0; j < existing_paths_id.size(); ++j) {
      // compare with one path
      bool same_topo = IsHomotopy(paths[i], paths[existing_paths_id[j]], vg, -1,
                                  max_dist_raycast, false);

      if (same_topo) {
        new_path = false;
        break;
      }
    }

    if (new_path) {
      existing_paths_id.push_back(i);
    }
  }

  // save pruned paths
  for (int i = 0; i < existing_paths_id.size(); ++i) {
    pruned_paths.push_back(paths[existing_paths_id[i]]);
  }

  /* std::cout << "Pruned path num: " << pruned_paths.size() << std::endl; */

  return pruned_paths;
}

std::vector<std::vector<Eigen::Vector3d>>
SelectShortPaths(const std::vector<std::vector<Eigen::Vector3d>> &paths,
                 const unsigned int n_paths, const double ratio_to_short) {

  /* ---------- only reserve top short path ---------- */
  std::vector<std::vector<Eigen::Vector3d>> short_paths;
  std::vector<std::pair<double, int>> pair_vec;

  for (int i = 0; i < paths.size(); ++i) {
    pair_vec.push_back(std::make_pair(GetPathLength(paths[i]), i));
  }

  std::sort(pair_vec.begin(), pair_vec.end());

  double min_len = pair_vec.begin()->first;
  for (int i = 0; i < std::min(n_paths, (unsigned int)(paths.size())); ++i) {
    if (pair_vec[i].first / min_len < ratio_to_short) {
      short_paths.push_back(paths[pair_vec[i].second]);
    }
  }

  /* std::cout << "Select path num: " << short_paths.size() << std::endl; */

  return short_paths;
}

double GetPathProgress(const ::std::vector<double> point,
                       const ::std::vector<::std::vector<double>> path,
                       ::std::vector<double> &point_out,
                       double &proj_dist_out) {
  if (path.size() > 1) {
    // transform path to eigen vector
    ::std::vector<::Eigen::Vector3d> path_eig = VectorToEigen(path);
    ::Eigen::Vector3d point_eig(point[0], point[1], point[2]);

    // get projection of point on first segment to check if the progress is
    // negative or positive
    ::Eigen::Vector3d line_dir = path_eig[1] - path_eig[0];
    /* we need to find the point on the path that is the closest to the input
     point*/
    // start with the first point and compute the distance
    ::Eigen::Vector3d dist_vec = point_eig - path_eig[0];
    double dist_min = dist_vec.norm();
    int path_idx = 1;
    ::Eigen::Vector3d curr_pt = path_eig[0];
    double progress = 0;
    double progress_final = 0;
    point_out = {curr_pt[0], curr_pt[1], curr_pt[2]};
    proj_dist_out = dist_min;
    while (path_idx < int(path.size())) {
      ::Eigen::Vector3d next_pt = path_eig[path_idx];
      ::Eigen::Vector3d diff = (next_pt - curr_pt);
      double dist_next = diff.norm();
      double samp_dist = 0.01;
      if (dist_next > samp_dist) {
        // sample the next point along the line between the current point
        // and the next point
        curr_pt = curr_pt + samp_dist * diff / dist_next;

        // upgate path progress
        progress = progress + samp_dist;
      } else {
        // set current point to the next path point
        curr_pt = next_pt;

        // increment path index and check if the next point is the last
        // point
        path_idx = path_idx + 1;

        // update path progress
        progress = progress + dist_next;
      }
      // check whether the distance is smaller than the previous distance
      dist_vec = point_eig - curr_pt;
      if (dist_vec.norm() < dist_min) {
        point_out = {curr_pt[0], curr_pt[1], curr_pt[2]};
        proj_dist_out = dist_vec.norm();
        dist_min = proj_dist_out;
        progress_final = progress;
      }
    }
    // return the minimum values
    return progress_final;
  } else {
    return 1;
  }
}

void WritePathToFile(const std::vector<Eigen::Vector3d> &path,
                     const std::string &file_name) {
  std::ofstream my_file;
  my_file.open(file_name);

  for (auto it : path) {
    my_file << it(0) << ", " << it(1) << ", " << it(2) << std::endl;
  }
  my_file.close();
}

void WriteMultiplePathsToFiles(
    const std::vector<std::vector<Eigen::Vector3d>> &paths,
    const std::string &file_name_prefix) {
  for (int i = 0; i < paths.size(); i++) {
    WritePathToFile(paths[i], file_name_prefix + std::to_string(i) + ".csv");
  }
}
} // namespace path_finding_util
