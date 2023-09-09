# - Config file for the convex_decomp_util package
# It defines the following variables
#  CONVEX_DECOMP_UTIL_INCLUDE_DIRS - include directories for decomp_util
#  CONVEX_DECOMP_UTIL_LIBRARIES    - libraries to link against

# Compute paths
get_filename_component(CONVEX_DECOMP_UTIL_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

set(CONVEX_DECOMP_UTIL_INCLUDE_DIRS "${CONVEX_DECOMP_UTIL_CMAKE_DIR}/../../../include/convex_decomp_util")

set(CONVEX_DECOMP_UTIL_LIBRARIES "${CONVEX_DECOMP_UTIL_CMAKE_DIR}/../../../lib/libconvex_decomp_util.a")
