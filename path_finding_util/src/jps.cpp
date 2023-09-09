// JPS implementation
#include "jps.hpp"

using namespace path_finding_util;

JPSPlanner::JPSPlanner(const ::voxel_grid_util::VoxelGrid &vg, bool verbose)
    : planner_verbose_(verbose), vg_(vg) {
  planner_verbose_ = verbose;
  if (planner_verbose_)
    printf("JPS PLANNER VERBOSE ON\n");
}

int JPSPlanner::Status() { return status_; }

std::vector<Eigen::Vector3d> JPSPlanner::GetPath() { return path_; }

std::vector<Eigen::Vector3d> JPSPlanner::GetRawPath() { return raw_path_; }

std::vector<Eigen::Vector3d>
JPSPlanner::RemoveCornerPts(const std::vector<Eigen::Vector3d> &path) {
  if (path.size() < 2)
    return path;

  // cut zigzag segment
  std::vector<Eigen::Vector3d> optimized_path;
  Eigen::Vector3d pose1 = path[0];
  Eigen::Vector3d pose2 = path[1];
  Eigen::Vector3d prev_pose = pose1;
  optimized_path.push_back(pose1);
  double cost1, cost2, cost3;

  if (IsLineClear(pose1, pose2, vg_, 100))
    cost1 = (pose1 - pose2).norm();
  else
    cost1 = std::numeric_limits<double>::infinity();

  for (unsigned int i = 1; i < path.size() - 1; i++) {
    pose1 = path[i];
    pose2 = path[i + 1];
    if (IsLineClear(pose1, pose2, vg_, 100))
      cost2 = (pose1 - pose2).norm();
    else
      cost2 = std::numeric_limits<double>::infinity();

    if (IsLineClear(prev_pose, pose2, vg_, 100))
      cost3 = (prev_pose - pose2).norm();
    else
      cost3 = std::numeric_limits<double>::infinity();

    if (cost3 < cost1 + cost2)
      cost1 = cost3;
    else {
      optimized_path.push_back(path[i]);
      cost1 = (pose1 - pose2).norm();
      prev_pose = pose1;
    }
  }

  optimized_path.push_back(path.back());
  return optimized_path;
}

std::vector<Eigen::Vector3d>
JPSPlanner::RemoveLinePts(const std::vector<Eigen::Vector3d> &path) {
  if (path.size() < 3)
    return path;

  std::vector<Eigen::Vector3d> new_path;
  new_path.push_back(path.front());
  for (unsigned int i = 1; i < path.size() - 1; i++) {
    Eigen::Vector3d p = (path[i + 1] - path[i]) - (path[i] - path[i - 1]);
    if (fabs(p(0)) + fabs(p(1)) + fabs(p(2)) > 1e-2)
      new_path.push_back(path[i]);
  }
  new_path.push_back(path.back());
  return new_path;
}
