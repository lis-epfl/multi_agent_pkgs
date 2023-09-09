// global planner class
#include "jps.hpp"
#include "prm.hpp"
#include "voxel_grid.hpp"

namespace path_finding_util {

class GlobalPlanner {
public:
  /**
   * @brief plan using the prm planner
   * @param start: start vector in the grid frame
   * @param goal: goal vector in the grid frame
   * @param vg: voxel grid
   * @param sample_inflate: inflation dimensions for sampling
   * @param max_sample_time: maximum sampling time
   * @param max_dist_raycast: maximum raycasting distance when checking for
   *        homotopies
   * @param max_raw_path: maximum raw paths to find from depth first search of
   *        the Graph
   * @param max_raw_path_2: maximum raw paths for choosing paths with the least
   *        number of nodes
   * @param max_short_path: maximum number of short paths after shortening the
   *        raw paths
   * @param ratio_to_short: the ration of the
   * @return the shortest N paths found from start to goal
   */
  std::vector<std::vector<Eigen::Vector3d>>
  PlanPRM(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
          const ::voxel_grid_util::VoxelGrid *vg,
          const Eigen::Vector3d &sample_inflate, const double max_sample_time,
          const double max_sample_num, const double max_dist_raycast,
          const unsigned int max_raw_path, const unsigned int max_raw_path_2,
          const unsigned int max_short_path, const double ratio_to_short) const;

  /**
   * @brief plan using A* planner
   * @param start: start vector in the grid frame
   * @param goal: goal vector in the grid frame
   * @param vg: voxel grid that contains obstacle information
   * @param max_expand maximum number of expansion allowed, optional, default
   *        is -1, means no limitation
   * @param weight_heur weight of heuristic, optional, default as 1
   * @param verbose flag for printing debug info, optional, default as False
   * @return the path found by A*
   */
  std::vector<Eigen::Vector3d>
  PlanAStar(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
            const ::voxel_grid_util::VoxelGrid *vg, const int max_expand = 1,
            const double weight_heur = 1, const bool verbose = true) const;

  /**
   * @brief plan using A* planner with artificial potential fields
   * @param start: start vector in the grid frame
   * @param goal: goal vector in the grid frame
   * @param vg: voxel grid that contains obstacle information
   * @param apf_coeff: the coefficent to calculate the cost of a voxel close to
   *        an obstacle (apf_coeff)*1/distance^(apf_degree); default = 2
   * @param apf_degree: the degree in the cost function
   *        (apf_coeff)*1/distance^(apf_degree); default = 1;
   * @param apf_dist: maximal distance (in number of voxels) with which to check
   *        the closest obstacles for adding a cost; default = 1
   * @param weight_heur weight of heuristic, optional, default as 1
   * @param verbose flag for printing debug info, optional, default as False
   * @param max_expand maximum number of expansion allowed, optional, default
   *        is -1, means no limitation
   * @return the path found by A*
   */
  std::vector<Eigen::Vector3d>
  PlanAStarAPF(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
               const ::voxel_grid_util::VoxelGrid *vg,
               const double apf_coeff = 2, const unsigned int apf_degree = 1,
               const unsigned int apf_dist = 1, const int max_expand = -1,
               const double weight_heur = 1, const bool verbose = true) const;

  /**
   * @brief plan using JPS planner
   * @param start: start vector in the grid frame
   * @param goal: goal vector in the grid frame
   * @param vg: voxel grid that contains obstacle information
   * @param max_expand maximum number of expansion allowed, optional, default
   *        is -1, means no limitation
   * @param weight_heur weight of heuristic, optional, default as 1
   * @param verbose flag for printing debug info, optional, default as False
   * @return the path found by JPS
   */
  std::vector<Eigen::Vector3d>
  PlanJPS(const Eigen::Vector3d &start, const Eigen::Vector3d goal,
          const ::voxel_grid_util::VoxelGrid *vg, const int max_expand = 1,
          const double weight_heur = 1, const bool verbose = true);
};

} // namespace path_finding_util
