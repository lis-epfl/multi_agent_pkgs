# - Config file for the convex_decomp_util package
# It defines the following variables
#  VOXEL_GRID_UTIL_INCLUDE_DIRS - include directories for decomp_util
#  VOXEL_GRID_UTIL_LIBRARIES - libraries to link against

# Compute paths
get_filename_component(VOXEL_GRID_UTIL_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

set(VOXEL_GRID_UTIL_INCLUDE_DIRS "${VOXEL_GRID_UTIL_CMAKE_DIR}/../../../include/voxel_grid_util")

set(VOXEL_GRID_UTIL_LIBRARIES "${VOXEL_GRID_UTIL_CMAKE_DIR}/../../../lib/libvoxel_grid_util.a")
