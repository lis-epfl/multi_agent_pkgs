#include "raycast.hpp"

namespace voxel_grid_util {
int signum(int x) { return x == 0 ? 0 : x < 0 ? -1 : 1; }

double mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

double intbound(double s, double ds) {
  // find the smallest positive t such that s+t*ds is an integer.
  if (ds < 0) {
    return intbound(-s, -ds);
  } else {
    s = mod(s, 1);
    // problem is now s+t*ds = 1
    return (1 - s) / ds;
  }
}

::std::vector<::Eigen::Vector3d>
Raycast(const ::Eigen::Vector3d &start, const ::Eigen::Vector3d &end,
        ::Eigen::Vector3d &collision_pt, const VoxelGrid &vg,
        const double max_dist, const bool verbose) {
  // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
  // by John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/vieswdoc/summary?doi=10.1.1.42.3443>
  // Extensions to the described algorithm:
  //   • Imposed a distance limit.
  //   • The face passed through to reach the current cube is provided to
  //     the callback.

  // The foundation of this algorithm is a parameterized representation of
  // the provided ray,
  //                    origin + t * direction,
  // except that t is not actually stored; rather, at any given point in the
  // traversal, we keep track of the *greater* t values which we would have
  // if we took a step sufficient to cross a cube boundary along that axis
  // (i.e. change the integer part of the coordinate) in the variables
  // tMaxX, tMaxY, and tMaxZ.
  // we modified the initial algorithm to make it raycast exactly in the point
  // direction and not in the voxel direction

  // Declare output vector
  std::vector<Eigen::Vector3d> output;

  // define offset to get the middle of the voxels
  // Eigen::Vector3d offset_vox(0.5, 0.5, 0.5);
  // Cube containing origin point.
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  Eigen::Vector3d direction = (end - start);
  double max_dist_squared = max_dist * max_dist;

  // Break out direction vector.
  double dx_int = endX - x;
  double dy_int = endY - y;
  double dz_int = endZ - z;
  double dx = end.x() - start.x();
  double dy = end.y() - start.y();
  double dz = end.z() - start.z();

  Eigen::Vector3d dir(dx, dy, dz);

  // Direction to increment x,y,z when stepping.
  int stepX = (int)signum((int)dx_int);
  int stepY = (int)signum((int)dy_int);
  int stepZ = (int)signum((int)dz_int);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  // The change in t when taking a step (always positive).
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  // Avoids an infinite loop.
  if (stepX == 0 && stepY == 0 && stepZ == 0) {
    output.push_back(end);
    output.push_back(start);
    return output;
  }

  double dist = 0;
  double tMax = 0;
  if (verbose) {
    std::cout << "------------------- new raycast --------------------"
              << std::endl;
  }
  while (true) {
    Eigen::Vector3i curr_point_i(x, y, z);

    // compute real point of intersection
    // save global tMax and do origin + tMax*dir
    Eigen::Vector3d curr_point_real = start + std::min(1.0, tMax) * dir;

    if (verbose) {
      std::cout << std::endl;
      std::cout << "curr_point: " << curr_point_i.transpose() << std::endl;
      std::cout << "end x,y,z : " << endX << " " << endY << " " << endZ
                << std::endl;
      std::cout << "tMax x,y,z: " << tMaxX << " " << tMaxY << " " << tMaxZ
                << std::endl;
      std::cout << "tMax: " << tMax << std::endl;
      std::cout << "step x,y,z: " << stepX << " " << stepY << " " << stepZ
                << std::endl;
      std::cout << "dx, dy, dz: " << dx << " " << dy << " " << dz << std::endl;

      std::cout << "curr_point_real: " << curr_point_real.transpose()
                << std::endl;
    }

    if (vg.IsInsideGridInt(curr_point_i)) {
      if (vg.IsOccupied(curr_point_i) && tMax <= 1) {
        // set collision point and break
        collision_pt = curr_point_real; // + offset_vox;
        if (verbose) {
          std::cout << "detected collision: " << collision_pt.transpose()
                    << std::endl;
        }
        output.push_back(collision_pt);
        break;
      }
      output.push_back(curr_point_real);

      dist = (Eigen::Vector3d(x, y, z) - start).squaredNorm();

      if (dist > max_dist_squared) {
        if (verbose) {
          std::cout << "max distance reached" << std::endl;
        }
        break;
      }

      if (output.size() > 1500) {
        std::cerr << "Error, too many racyast voxels." << std::endl;
        throw std::out_of_range("Too many raycast voxels");
      }
    }

    if (tMax >= 1) {
      // only add if you wanna test real raycasting
      break;
    }

    // tMaxX stores the t-value at which we cross a cube boundary along the
    // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
    // chooses the closest cube boundary. Only the first case of the four
    // has been commented in detail.
    if (tMaxX < tMaxY && stepX != 0 || stepY == 0) {
      if (tMaxX < tMaxZ && stepX != 0 || stepZ == 0) {
        // Update tMax global
        tMax = tMaxX;
        // Update which cube we are now in.
        x += stepX;
        // Adjust tMaxX to the next X-oriented boundary crossing.
        tMaxX += tDeltaX;
      } else {
        tMax = tMaxZ;
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ && stepY != 0 || stepZ == 0) {
        tMax = tMaxY;
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        tMax = tMaxZ;
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }

  return output;
}
} // namespace voxel_grid_util
