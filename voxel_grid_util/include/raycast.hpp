#ifndef VOXEL_GRID_UTIL_RAYCAST_H_
#define VOXEL_GRID_UTIL_RAYCAST_H_

#include "voxel_grid.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace voxel_grid_util {

// return the sign of a number
double signum(double x);

// find the modulus of a value
double mod(double value, double modulus);

// find the smallest positive t such that s+t*ds is an integer.
double intbound(double s, double ds);

/**
 * @brief raycast between 2 points of the path and return raycasted path in a
 *        grid of voxel size 1.
 * @param pt_start: the starting point of the raycast in the grid frame
 *        pt_end: the end point of the raycast in the grid frame
 *        collision_pt: collision_pt
 *        vg: voxel/occupancy grid
 *        verbose: if true, std::cout some variables
 * @return raycasted points coord (the intersection with the voxels)
 */
std::vector<Eigen::Vector3d>
Raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
        Eigen::Vector3d &collision_pt, const VoxelGrid &vg,
        const double max_dist, const bool verbose = false);

} // namespace voxel_grid_util
#endif
