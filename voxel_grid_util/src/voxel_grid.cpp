#include "voxel_grid.hpp"
#include <cstdint>
#include <iterator>
#include <vector>

namespace voxel_grid_util {
VoxelGrid::VoxelGrid(Eigen::Vector3d &origin, Eigen::Vector3i &dim,
                     double vox_size, bool free)
    : origin_(origin), dim_(dim), vox_size_(vox_size) {
  data_size_ = dim_(0) * dim_(1) * dim_(2);
  if (!free) {
    data_.resize(data_size_, ENV_BUILDER_UNK);
  } else {
    data_.resize(data_size_, ENV_BUILDER_FREE);
  }
}

VoxelGrid::VoxelGrid(Eigen::Vector3d &origin, Eigen::Vector3d &dim,
                     double vox_size, bool free)
    : origin_(origin), vox_size_(vox_size) {
  dim_(0) = ceil(dim(0) / vox_size);
  dim_(1) = ceil(dim(1) / vox_size);
  dim_(2) = ceil(dim(2) / vox_size);

  data_size_ = dim_(0) * dim_(1) * dim_(2);
  if (!free) {
    data_.resize(data_size_, ENV_BUILDER_UNK);
  } else {
    data_.resize(data_size_, ENV_BUILDER_FREE);
  }
}

VoxelGrid::VoxelGrid(Eigen::Vector3d &origin, Eigen::Vector3i &dim,
                     double vox_size, std::vector<voxel_data_type> &data)
    : origin_(origin), dim_(dim), vox_size_(vox_size), data_(data) {
  data_size_ = dim_(0) * dim_(1) * dim_(2);
}

bool VoxelGrid::IsInsideGridInt(const Eigen::Vector3i &coord_int) const {
  if (coord_int(0) < dim_(0) && coord_int(1) < dim_(1) &&
      coord_int(2) < dim_(2) && coord_int(0) >= 0 && coord_int(1) >= 0 &&
      coord_int(2) >= 0) {
    return true;
  } else {
    return false;
  }
}

bool VoxelGrid::IsInsideGridInt(const Eigen::Vector3d &coord_int) const {
  return IsInsideGridInt(
      Eigen::Vector3i(coord_int(0), coord_int(1), coord_int(2)));
}

bool VoxelGrid::IsInsideGridLocal(const Eigen::Vector3d &coord_dbl) const {
  Eigen::Vector3i coord_int((int)(coord_dbl(0) / vox_size_),
                            (int)(coord_dbl(1) / vox_size_),
                            (int)(coord_dbl(2) / vox_size_));
  return IsInsideGridInt(coord_int);
}

bool VoxelGrid::IsInsideGridGlobal(const Eigen::Vector3d &coord_global) const {
  Eigen::Vector3d coord_local = coord_global - origin_;
  return IsInsideGridLocal(coord_local);
}

int VoxelGrid::CoordToIdx(const Eigen::Vector3i &coord) const {
  int idx;
  if (IsInsideGridInt(coord)) {
    idx = coord(0) + coord(1) * dim_(0) + coord(2) * dim_(0) * dim_(1);
  } else {
    idx = -1;
  }
  return idx;
}

Eigen::Vector3i VoxelGrid::IdxToCoord(const unsigned int idx) const {
  int z = idx / (dim_(0) * dim_(1));
  int y = (idx - z * dim_(0) * dim_(1)) / dim_(0);
  int x = (idx - z * dim_(0) * dim_(1) - y * dim_(0));
  return Eigen::Vector3i(x, y, z);
}

bool VoxelGrid::SetVoxelInt(const Eigen::Vector3i &coord_int,
                            const voxel_data_type val) {
  int idx = CoordToIdx(coord_int);
  if (idx != -1) {
    data_[idx] = val;
    return true;
  } else {
    return false;
  }
}

bool VoxelGrid::SetVoxelDouble(const Eigen::Vector3d &coord_dbl,
                               const voxel_data_type val) {
  Eigen::Vector3i coord_int((int)(coord_dbl(0) / vox_size_),
                            (int)(coord_dbl(1) / vox_size_),
                            (int)(coord_dbl(2) / vox_size_));
  return SetVoxelInt(coord_int, val);
}

bool VoxelGrid::SetVoxelGlobal(const Eigen::Vector3d &coord_global,
                               const voxel_data_type val) {
  Eigen::Vector3d coord_local = coord_global - origin_;
  return SetVoxelDouble(coord_local, val);
}

voxel_data_type VoxelGrid::GetVoxelInt(const Eigen::Vector3i &coord_int) const {
  int idx = CoordToIdx(coord_int);
  if (idx != -1) {
    return data_[idx];
  } else {
    return -1;
  }
}

voxel_data_type VoxelGrid::GetVoxelInt(const Eigen::Vector3d &coord_int) const {
  return GetVoxelInt(Eigen::Vector3i(coord_int(0), coord_int(1), coord_int(2)));
}

voxel_data_type
VoxelGrid::GetVoxelDouble(const Eigen::Vector3d &coord_dbl) const {
  Eigen::Vector3i coord_int((int)(coord_dbl(0) / vox_size_),
                            (int)(coord_dbl(1) / vox_size_),
                            (int)(coord_dbl(2) / vox_size_));
  return GetVoxelInt(coord_int);
}

voxel_data_type
VoxelGrid::GetVoxelGlobal(const Eigen::Vector3d &coord_global) const {
  Eigen::Vector3d coord_local = coord_global - origin_;
  return GetVoxelDouble(coord_local);
}

Eigen::Vector3d
VoxelGrid::GetCoordLocal(const Eigen::Vector3d &coord_global) const {
  Eigen::Vector3d pt_out = coord_global - origin_;
  pt_out = pt_out / vox_size_;
  return pt_out;
}

Eigen::Vector3d
VoxelGrid::GetCoordGlobal(const Eigen::Vector3d &coord_local) const {
  Eigen::Vector3d pt_out = coord_local * vox_size_ + origin_;
  return pt_out;
}

bool VoxelGrid::IsOccupied(const Eigen::Vector3i coord_int) const {
  if (GetVoxelInt(coord_int) == ENV_BUILDER_OCC)
    return true;
  else
    return false;
}

bool VoxelGrid::IsOccupied(const Eigen::Vector3d coord_int) const {
  if (GetVoxelInt(coord_int) == ENV_BUILDER_OCC)
    return true;
  else
    return false;
}

bool VoxelGrid::IsFree(const Eigen::Vector3i coord_int) const {
  if (GetVoxelInt(coord_int) == ENV_BUILDER_FREE)
    return true;
  else
    return false;
}

bool VoxelGrid::IsFree(const Eigen::Vector3d coord_int) const {
  if (GetVoxelInt(coord_int) == ENV_BUILDER_FREE)
    return true;
  else
    return false;
}

::std::vector<::std::pair<::Eigen::Vector3i, int8_t>>
VoxelGrid::CreateMask(double mask_dist, double pow) {
  // create mask variable
  ::std::vector<::std::pair<::Eigen::Vector3i, int8_t>> mask;
  double res = vox_size_;
  double h_max = ENV_BUILDER_OCC;
  int rn = ::std::ceil(mask_dist / res);
  ::Eigen::Vector3i n;

  if (mask_dist > 0) {
    for (n(0) = -rn; n(0) <= rn; n(0)++) {
      for (n(1) = -rn; n(1) <= rn; n(1)++) {
        for (n(2) = -rn; n(2) <= rn; n(2)++) {
          double dist = ::std::hypot(::std::hypot(n(0), n(1)), n(2));
          // remove one voxel distance from the computed distance
          dist = ::std::abs(dist - 1);
          // check if the distance is smaller than the mask_dist
          if (dist * res >= mask_dist)
            continue;
          double h =
              h_max * ::std::pow((1 - (double)::std::hypot(
                                          ::std::hypot(n(0), n(1)), n(2)) /
                                          (rn + 1)),
                                 pow);
          if (h > 1e-3)
            mask.push_back(::std::make_pair(n, (int8_t)h));
        }
      }
    }
  }

  return mask;
}

void VoxelGrid::InflateObstacles(double inflation_dist) {
  // first create mask to get all the voxels that are within the inflation
  // distance of the center voxel that is being considered for inflation
  ::std::vector<::std::pair<::Eigen::Vector3i, int8_t>> mask =
      CreateMask(inflation_dist, 1);

  // create vector to save occupied voxels
  ::std::vector<::Eigen::Vector3i> occ_voxels;

  // set the voxels that are within that distance to occupied
  ::Eigen::Vector3i n;
  for (n(0) = 0; n(0) < dim_(0); n(0)++) {
    for (n(1) = 0; n(1) < dim_(1); n(1)++) {
      for (n(2) = 0; n(2) < dim_(2); n(2)++) {
        if (IsOccupied(n)) {
          occ_voxels.push_back(n);
        }
      }
    }
  }

  for (auto &n : occ_voxels) {
    for (const auto &it : mask) {
      const ::Eigen::Vector3i new_n = n + it.first;
      SetVoxelInt(new_n, ENV_BUILDER_OCC);
    }
  }
}

void VoxelGrid::CreatePotentialField(double potential_dist, int pow) {}

Eigen::Vector3d VoxelGrid::GetOrigin() const { return origin_; }

Eigen::Vector3i VoxelGrid::GetDim() const { return dim_; }

Eigen::Vector3d VoxelGrid::GetRealDim() const {
  return Eigen::Vector3d(dim_(0) * vox_size_, dim_(1) * vox_size_,
                         dim_(2) * vox_size_);
}

double VoxelGrid::GetVoxSize() const { return vox_size_; }

unsigned int VoxelGrid::GetDataSize() const { return data_size_; }

std::vector<voxel_data_type> VoxelGrid::GetData() const { return data_; }

void AddObstacle(VoxelGrid::Ptr vg, const Eigen::Vector3d &center_obs,
                 const Eigen::Vector3d &dim_obs) {
  Eigen::Vector3i start_idx;
  Eigen::Vector3i end_idx;
  double voxel_size = vg->GetVoxSize();

  for (int i = 0; i < 3; i++) {
    start_idx(i) = std::floor((center_obs(i) - dim_obs(i) / 2) / voxel_size);
    end_idx(i) = std::floor((center_obs(i) + dim_obs(i) / 2) / voxel_size);
  }

  for (int i = start_idx(0); i <= end_idx(0); i++) {
    for (int j = start_idx(1); j <= end_idx(1); j++) {
      for (int k = start_idx(2); k <= end_idx(2); k++) {
        vg->SetVoxelInt(Eigen::Vector3i(i, j, k), ENV_BUILDER_OCC);
      }
    }
  }
}

void WriteGridToFile(VoxelGrid::Ptr vg, std::string file_name) {
  std::ofstream my_file;
  my_file.open(file_name);
  Eigen::Vector3i dim = vg->GetDim();
  double vox_size = vg->GetVoxSize();
  Eigen::Vector3d origin = vg->GetOrigin();

  for (int i = 0; i < dim(0); i++) {
    for (int j = 0; j < dim(1); j++) {
      for (int k = 0; k < dim(2); k++) {
        if (vg->GetVoxelInt(Eigen::Vector3i(i, j, k)) == ENV_BUILDER_OCC) {
          my_file << (i + 0.5) * vox_size + origin(0) << ", "
                  << (j + 0.5) * vox_size + origin(1) << ", "
                  << (k + 0.5) * vox_size + origin(2) << std::endl;
        }
      }
    }
  }
  my_file.close();
}
} // namespace voxel_grid_util
