// JPS header file
#pragma once

#include "graph_search.hpp"
#include "path_tools.hpp"
#include "voxel_grid.hpp"

#include <Eigen/Dense>

namespace path_finding_util {
class JPSPlanner {
public:
  /**
   * @brief Simple constructor
   * @param verbose enable debug mode
   */
  JPSPlanner(const ::voxel_grid_util::VoxelGrid &vg, bool verbose = false);

  /**
   * @brief Status of the planner
   *
   * 0 --- exit normally;
   * -1 --- no path found;
   * 1, 2 --- start or goal is not free.
   */
  int Status();

  // get the modified path
  std::vector<Eigen::Vector3d> GetPath();

  // get the raw path
  std::vector<Eigen::Vector3d> GetRawPath();

  // remove redundant points on the same line
  std::vector<Eigen::Vector3d>
  RemoveLinePts(const std::vector<Eigen::Vector3d> &path);

  // remove some corner waypoints
  std::vector<Eigen::Vector3d>
  RemoveCornerPts(const std::vector<Eigen::Vector3d> &path);

  /**
   * @brief planning function
   * @param start: starting point
   * @param goal: goal point
   * @param weight_heur: weight of the heuristic of A*; default 1
   * @param use_jps: whether to use JPS or not; default true
   * @return false: if we didn't find a path
   */
  bool Plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
            double weight_heur = 1, bool use_jps = true);

  // get the nodes in open set
  std::vector<Eigen::Vector3d> GetOpenSet() const;

  // get the nodes in close set
  std::vector<Eigen::Vector3d> GetCloseSet() const;

  // get all the nodes
  std::vector<Eigen::Vector3d> GetAllSet() const;

private:
  // voxel grid that contains obstacles information
  const ::voxel_grid_util::VoxelGrid &vg_;

  // assume using 3D voxel map for all planning
  std::shared_ptr<GraphSearch> graph_search_;

  // raw path from planner
  std::vector<Eigen::Vector3d> raw_path_;

  // modified path for future usage
  std::vector<Eigen::Vector3d> path_;

  // flag indicating the success of planning
  int status_ = 0;

  // enabled for printing info
  bool planner_verbose_;
};
} // namespace path_finding_util
