#pragma once

#include "raycast.hpp"
#include "voxel_grid.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <math.h>
#include <vector>

// used check if we reached a path point when discretizing the path in
// the function DiscretizePathStep()
#define PATH_FINDING_UTIL_COMP_PREC 1e-3

// define path tools useful for both JPS and PRM (or any path for that matter)
// grid frame: to get to this frame, we subtract the point/path by the grid
//             origin and then  divide by the voxel size. It will be used to
//             check IsLineClear and IsHomotopy
namespace path_finding_util {

// transform path from vector to Eigen
::std::vector<::Eigen::Vector3d>
VectorToEigen(::std::vector<::std::vector<double>> path);

// transform path from Eigen to Vector
::std::vector<::std::vector<double>>
EigenToVector(::std::vector<::Eigen::Vector3d> path);

/**
 * @brief get the length of the path
 **/
double GetPathLength(const std::vector<Eigen::Vector3d> &path);

/**
 * @brief Transform path to grid frame i.e. remove the origin and divide by the
 *        voxel size
 * @param path: path to transform
 *        vg: voxel grid
 * @return return transformed path
 */
std::vector<Eigen::Vector3d>
PathGlobalToLocal(const std::vector<Eigen::Vector3d> &path,
                  const ::voxel_grid_util::VoxelGrid &vg);

/**
 * @brief Transform path to global frame i.e. multiply by the
 *        voxel size and add the origin
 * @param path: path to transform
 *        vg: voxel grid
 * @return return transformed path
 */
std::vector<Eigen::Vector3d>
PathLocalToGlobal(const std::vector<Eigen::Vector3d> &path,
                  const ::voxel_grid_util::VoxelGrid &vg);

/**
 * @brief discretize a given path (series of points) into another series of
 *        point with equal step distance between them
 * @param path: the path we wanna discretize
 *        dis_step: discretization step of the path
 *        keep_orig: keep the nodes of path in the final discretized path
 * @return path_dis: discretized path
 */
std::vector<Eigen::Vector3d>
DiscretizePathStep(const std::vector<Eigen::Vector3d> &path,
                   const double dis_step, const bool keep_orig = false);

/**
 * @brief discretize a given path (series of points) into another series of
 *        point with equal step distance between them
 * @param path: the path we wanna discretize
 *        n_points: number of points in the path
 *        keep_orig: keep the nodes of path in the final discretized path
 * @return dis_path: discretized path
 */
std::vector<Eigen::Vector3d>
DiscretizePathPoints(const std::vector<Eigen::Vector3d> &path,
                     const int n_points, const bool keep_orig = false);

/**
 * @brief raycast between 2 points of the path and find collision.
 * @details raycast between 2 points and find the collision if it exists; all
 *          variables are in the grid frame i.e. as if voxel size = 1.
 * @param pt_start: the starting point of the raycast in the grid frame
 *        pt_end: the end point of the raycast in the grid frame
 *        vg: voxel/occupancy grid
 *        max_dist_raycast: maximum raycasting distance
 *        verbose: std::cout variables
 * @return true if there are no obstacles between the start and the end of the
 *         raycast.
 *         fasle otherwise
 */
bool IsLineClear(const Eigen::Vector3d &pt_start, const Eigen::Vector3d &pt_end,
                 const ::voxel_grid_util::VoxelGrid &vg,
                 const double max_dist_raycast, const bool verbose = false);
/**
 * @brief raycast between 2 points of the path and find collision.
 * @details raycast between 2 points and find the collision if it exists; all
 *          variables are in the grid frame i.e. as if voxel size = 1.
 * @param pt_start: the starting point of the raycast in the grid frame
 *        pt_end: the end point of the raycast in the grid frame
 *        vg: voxel/occupancy grid
 *        max_dist_raycast: maximum raycasting distance
 *        collision_pt: collision point output in the raycast (grid frame)
 *        visited_points: visited points output in the raycast (grid frame)
 *        verbose: std::cout variables
 * @return true if there are no obstacles between the start and the end of the
 *         raycast.
 *         false otherwise
 */
bool IsLineClear(const Eigen::Vector3d &pt_start, const Eigen::Vector3d &pt_end,
                 const ::voxel_grid_util::VoxelGrid &vg,
                 const double max_dist_raycast, Eigen::Vector3d &collision_pt,
                 std::vector<Eigen::Vector3d> &visited_points,
                 const bool verbose = false);

/**
 * @brief shorten a given path by raycasting until we hit an obstacle
 * @details we discretize the path then check if the line is clear from the
 *          start point to the 2nd discrete point. If yes, continue to the next
 *          discrete point. Continue until we hit an obstacle. When this
 *          happens, add the previous/last discrete point that has a clear line
 *          to the start to the final path and set it as the start point.
 *          Continue with this algorithm until we reach the final point.
 * @param path: the path we wanna discretize in the grid framegt
 *        vg: voxel/occupancy grid
 *        dis_step: discretization step of the path. 1 should suffice since we
 *                  are in the grid frame
 *        max_dist_raycast: maximum raycasting distance
 *        verbose: std::cout variables
 * @return short_path: shortened path
 */
std::vector<Eigen::Vector3d>
ShortenPath(const std::vector<Eigen::Vector3d> &path,
            const ::voxel_grid_util::VoxelGrid &vg, const double dis_step,
            const double max_dist_raycast, const bool verbose = false);

// shorten the distance mapper path; take a voxel that is not in the potential
// field and then find the next voxel that is not in the potential field, then
// see if we shorten it, do we hit a voxel of the potential field; if no hit,
// then we shorten
::std::vector<::std::vector<double>>
ShortenDMPPath(::std::vector<::std::vector<double>> &path,
               ::voxel_grid_util::VoxelGrid &vg); 

/**
 * @brief check if 2 paths are equivalent i.e. a homotopy in the grid frame
 * @param path_1: the first path in the grid frame i.e. -origin /voxel_size
 *        path_2: the second path in the grid frame i.e. -origin /voxel_size
 *        vg: voxel/occupancy grid
 *        n_points: the number of points to test on the paths. If it is
 *                  negative, we compute inside the function the number of
 *                  discretization point using max_length_path/voxel_size
 *        max_dist_raycast: maximum raycasting distance
 *        verbose: std::cout variables
 * @return true if the the 2 paths are a homotopy
 *         false otherwise
 */
bool IsHomotopy(const std::vector<Eigen::Vector3d> &path_1,
                const std::vector<Eigen::Vector3d> &path_2,
                const ::voxel_grid_util::VoxelGrid &vg, const int n_points,
                const double max_dist_raycast, const bool verbose = false);

/**
 * @brief prune equivalent paths that are homotopies
 * @param paths: paths to prune
 * @return return pruned paths
 */
std::vector<std::vector<Eigen::Vector3d>>
PruneEquivalentPaths(const std::vector<std::vector<Eigen::Vector3d>> &paths,
                     const ::voxel_grid_util::VoxelGrid &vg,
                     const double max_dist_raycast);

/**
 * @brief select N shortest paths
 * @param paths: vector containing the paths
 *        n_paths: number of paths to select
 *        ratio_to_short: ratio to the shortest path that other paths should
 *                        not exceed in order to be added to the return vector
 * @return vector containing N shortest paths
 */
std::vector<std::vector<Eigen::Vector3d>>
SelectShortPaths(const std::vector<std::vector<Eigen::Vector3d>> &paths,
                 const unsigned int n_paths, const double ratio_to_short);

/**
 * @brief get path progess
 * @param point: the point which we want to see its progress
 *        path: path along which to check the progress
 *        point_out: the point that is the closest projection of point on the
                     path
 *        proj_dist_out: the distance between the point and the path
 * @return: return the progress along the path (can be negative or positive)
 */
double GetPathProgress(const ::std::vector<double> point,
                       const ::std::vector<::std::vector<double>> path,
                       ::std::vector<double> &point_out, double &proj_dist_out);

/**
 * @brief write path to csv file with each point on a row
 * @param path: path to save
 *         file_name: file name
 */
void WritePathToFile(const std::vector<Eigen::Vector3d> &path,
                     const std::string &file_name);

/**
 * @brief write multiple paths to multiple csv files with each point on a row
 * @param paths: paths to save
 *        file_name_prefix: file name prefix
 */
void WriteMultiplePathsToFiles(
    const std::vector<std::vector<Eigen::Vector3d>> &paths,
    const std::string &file_name_prefix);

} // namespace path_finding_util
