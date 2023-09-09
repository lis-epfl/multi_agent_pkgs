#ifndef CONVEX_DECOMP_UTIL_CONVEX_DECOMP_H_
#define CONVEX_DECOMP_UTIL_CONVEX_DECOMP_H_

#include <decomp_geometry/geometric_utils.h>
#include <decomp_geometry/polyhedron.h>
#include <decomp_util/seed_decomp.h>

#include <deque>

#define CVX_DCMP_FREE 0
#define CVX_DCMP_OCC 100

namespace convex_decomp_lib {

// data type for the voxel grid
typedef int8_t data_type;

// define vector that contains 4 integers
typedef Veci<4> Vec4i;

// define corner structure to be added to the convex polyhedron when there is a
// slope in the environment
struct Corner3D {
  Vec3f position;
  int slope;
  int direction;
  bool fixed;
  int steps;

  Corner3D() {
    slope = 0;
    direction = -1; // 0 for horiz, 1 for vert
    position = Vec3f(0, 0, 0);
    fixed = false;
    steps = 0;
  }
};

// border that is used to define the borders of the convex grid
struct Border3D {
  std::vector<Vec3i> cells;
  Vec4i limits;
  Border3D() { limits = Vec4i(0, 0, 0, 0); }
};

// function to get the convex polyhedron around a seed by inflating an ellipsoid
// (decompUtil)
/* Polyhedron3D GetHyperplanes3DSeed(Vec3f, Vec3f); */

// function to get the convex polyhedron around a seed by inflating a convex
// grid
Polyhedron3D GetPolyOcta3D(Vec3i, std::vector<data_type> &, Vec3i, int, double, int,
                           Vec3f);

// function to get the convex polyhedron around a seed by inflating a
// shape-aware convex grid
Polyhedron3D GetPolyOcta3DNew(Vec3i seed, std::vector<data_type> &data,
                              Vec3i dim_3D, int n_it, double res_arg, int CONV,
                              Vec3f origin);

} // namespace convex_decomp_lib

#endif // CONVEX_DECOMP_UTIL_CONVEX_DECOMP_H_
