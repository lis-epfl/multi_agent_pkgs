#ifndef VOXEL_GRID_UTIL_VOXEL_GRID_H_
#define VOXEL_GRID_UTIL_VOXEL_GRID_H_

#include <Eigen/Dense>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#define ENV_BUILDER_OCC 100
#define ENV_BUILDER_FREE 0
#define ENV_BUILDER_UNK -1

namespace voxel_grid_util {

typedef int8_t voxel_data_type;

/**
 * @brief Voxel Grid class for testing path finding
 */
class VoxelGrid {

public:
  // define typedef for shared ptr; useful for aggregation in other classes
  typedef std::shared_ptr<VoxelGrid> Ptr;
  typedef std::shared_ptr<const VoxelGrid> ConstPtr;

  /**
   * @brief constructor of the voxel grid
   * @param origin: origin of the voxel grid
   * @param dim: dimension of the voxel grid in number of voxels
   * @param vox_size: voxel size of the voxel grid
   * @param free: if true, initialize the grid to free instead of unknown
   */
  VoxelGrid(Eigen::Vector3d &origin, Eigen::Vector3i &dim, double vox_size,
            bool free);

  /**
   * @brief constructor of the voxel grid
   * @param origin: origin of the voxel grid
   * @param dim: dimension of the voxel grid in meters
   * @param vox_size: voxel size of the voxel grid
   * @param free: if true, initialize the grid to free instead of unknown
   */
  VoxelGrid(Eigen::Vector3d &origin, Eigen::Vector3d &dim, double vox_size,
            bool free);

  /**
   * @brief constructor of the voxel grid
   * @param origin: origin of the voxel grid
   * @param dim: dimension of the voxel grid
   * @param vox_size: voxel size of the voxel grid
   * @param data: data to fill the voxel grid with
   */
  VoxelGrid(Eigen::Vector3d &origin, Eigen::Vector3i &dim, double vox_size,
            std::vector<voxel_data_type> &data);

  /**
   * @brief Check if a discrete point is inside the grid
   * @param coord_int: the point in the grid frame to check
   * @return true if inside the grid
   */
  bool IsInsideGridInt(const Eigen::Vector3i &coord_int) const;

  /**
   * @brief Check if a double point is inside the grid
   * @param coord_int: the point in the grid frame to check
   * @return true if inside the grid
   */
  bool IsInsideGridInt(const Eigen::Vector3d &coord_int) const;

  /**
   * @brief Check if a double point in the local frame is inside the grid
   * @details transform the point to the grid frame by dividing by the voxel
   *          size and then check if it is within the bounds of the grid
   * @param coord_int: the point in the local frame to check
   * @return true if inside the grid
   */
  bool IsInsideGridLocal(const Eigen::Vector3d &coord_dbl) const;

  /**
   * @brief Check if a double point in the global frame is inside the grid
   * @details transform the point to the grid frame by subtracting the origin
   *          and then dividing by the voxel size and then check if it is within
   *          the bounds of the grid
   * @param coord_int: the point in the local frame to check
   * @return true if inside the grid
   */
  bool IsInsideGridGlobal(const Eigen::Vector3d &coord_global) const;

  // get the voxel index for data_ from the coordinates
  // return -1 if the coordinate is outside the grid
  int CoordToIdx(const Eigen::Vector3i &coord) const;

  // get the coord of the point from its index
  Eigen::Vector3i IdxToCoord(const unsigned int idx) const;

  // set the voxel using the discrete coordinates of the grid (real
  // coordinates/res); the frame of  coord_int has the same origin as the
  // grid return false if the coordinates are outside the grid
  bool SetVoxelInt(const Eigen::Vector3i &coord_int, const voxel_data_type val);

  // set the voxel using the real coordinates of the grid (the frame of
  // coord_dbl has the same origin as the grid). return false if the coordinates
  // are outside the grid
  bool SetVoxelDouble(const Eigen::Vector3d &coord_dbl,
                      const voxel_data_type val);

  // set the voxel using the global coordinates
  bool SetVoxelGlobal(const Eigen::Vector3d &coord_global,
                      const voxel_data_type val);

  // get the voxel value from discrete coordinates (local)
  // return -1 if the coordinate is outside the grid
  voxel_data_type GetVoxelInt(const Eigen::Vector3i &coord_int) const;

  // get the voxel value from double coordinates in the grid frame (local)
  // return -1 if the coordinate is outside the grid
  voxel_data_type GetVoxelInt(const Eigen::Vector3d &coord_int) const;

  // get the voxel value from real coordinates (local)
  // return -1 if the coordinate is outside the grid
  voxel_data_type GetVoxelDouble(const Eigen::Vector3d &coord_dbl) const;

  // get the voxel value from real coordinates (global)
  // return -1 if the coordinate is outside the grid
  voxel_data_type GetVoxelGlobal(const Eigen::Vector3d &coord_global) const;

  // transform global coordinates to local coordinate (remove origin and
  // divide by voxel size)
  Eigen::Vector3d GetCoordLocal(const Eigen::Vector3d &coord_global) const;

  // transform local coordinates to global coordinate (multiply by voxel size
  // and then add the origin)
  Eigen::Vector3d GetCoordGlobal(const Eigen::Vector3d &coord_local) const;

  // test if voxel is occupied in the grid frame
  bool IsOccupied(const Eigen::Vector3i coord_int) const;

  // test if voxel is occupied in the grid frame
  bool IsOccupied(const Eigen::Vector3d coord_int) const;

  // test if voxel is free in the grid frame
  bool IsFree(const Eigen::Vector3i coord_int) const;

  // test if voxel is free in the grid frame
  bool IsFree(const Eigen::Vector3d coord_int) const;

  // inflate the obstacle of the grid by a certain distance
  void InflateObstacles(double inflation_dist);

  // create mask to get the voxels whithin a certain distance of a center voxel
  ::std::vector<::std::pair<::Eigen::Vector3i, int8_t>>
  CreateMask(double mask_dist, double pow);

  // get origin
  Eigen::Vector3d GetOrigin() const;

  // get dimensions
  Eigen::Vector3i GetDim() const;

  // get dimensions
  Eigen::Vector3d GetRealDim() const;

  // get voxel size
  double GetVoxSize() const;

  // get data size
  unsigned int GetDataSize() const;

  // get data
  std::vector<voxel_data_type> GetData() const;

private:
  // origin of the grid
  Eigen::Vector3d origin_;

  // discrete dimension of the grid
  Eigen::Vector3i dim_;

  // voxel size
  double vox_size_;

  // size of the data
  unsigned int data_size_;

  // the occupancy data of the grid
  std::vector<voxel_data_type> data_;
};

/**
 * @brief add a cuboid obstacle to the grid
 * @param center_obs: obstacle center
 *        dim_obs: obstacle dimensions
 */
void AddObstacle(VoxelGrid::Ptr vg, const Eigen::Vector3d &center_obs,
                 const Eigen::Vector3d &dim_obs);

/**
 * @brief write the occupied voxel centers to file (local frame). Each row
 *        contains the x, y, z coordintes
 * @param file_name: the name of the file to write "*.csv"
 */
void WriteGridToFile(VoxelGrid::Ptr vg, std::string file_name);

} // namespace voxel_grid_util

#endif // VOXEL_GRID_UTIL_VOXEL_GRID_H_
