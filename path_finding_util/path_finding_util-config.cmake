# - Config file for the convex_decomp_util package
# It defines the following variables
#  PATH_FINDING_UTIL_INCLUDE_DIRS - include directories for decomp_util
#  PATH_FINDING_UTIL_LIBRARIES - libraries to link against

# Compute paths
get_filename_component(PATH_FINDING_UTIL_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

set(PATH_FINDING_UTIL_INCLUDE_DIRS "${PATH_FINDING_UTIL_CMAKE_DIR}/../../../include/path_finding_util")

set(PATH_FINDING_UTIL_LIBRARIES "${PATH_FINDING_UTIL_CMAKE_DIR}/../../../lib/libpath_finding_util.a")
