// global planner implementation
#include "global_planner.hpp"

namespace path_finding_util {

std::vector<std::vector<Eigen::Vector3d>> GlobalPlanner::PlanPRM(
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
    const ::voxel_grid_util::VoxelGrid *vg,
    const Eigen::Vector3d &sample_inflate, const double max_sample_time,
    const double max_sample_num, const double max_dist_raycast,
    const unsigned int max_raw_path, const unsigned int max_raw_path_2,
    const unsigned int max_short_path, const double ratio_to_short) const {

  TopologyPRM topo_prm(sample_inflate, max_sample_time, max_sample_num,
                       max_dist_raycast, max_raw_path, max_raw_path_2,
                       max_short_path, ratio_to_short);

  Eigen::Vector3d start_local = vg->GetCoordLocal(start);
  Eigen::Vector3d goal_local = vg->GetCoordLocal(goal);

  topo_prm.FindTopoPaths(vg, start_local, goal_local);
  return topo_prm.GetFinalPaths();
}

std::vector<Eigen::Vector3d> GlobalPlanner::PlanAStar(
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
    const ::voxel_grid_util::VoxelGrid *vg, const int max_expand,
    const double weight_heur, const bool verbose) const {

  Eigen::Vector3d start_local = vg->GetCoordLocal(start);
  Eigen::Vector3d goal_local = vg->GetCoordLocal(goal);

  Eigen::Vector3i start_i(start_local[0], start_local[1], start_local[2]);
  Eigen::Vector3i goal_i(goal_local[0], goal_local[1], goal_local[2]);

  GraphSearch graph_search(vg->GetDim(), weight_heur, verbose);
  graph_search.Plan(vg, start_i, goal_i, false);
  std::vector<StatePtr> path = graph_search.GetPath();
  std::vector<Eigen::Vector3d> path_out =
      ::path_finding_util::ConvertPathToVector(path);

  std::vector<Eigen::Vector3d> path_out_final =
      ::path_finding_util::PathLocalToGlobal(path_out, *vg);

  // reverse the path
  ::std::reverse(path_out_final.begin(), path_out_final.end());

  return path_out;
}

std::vector<Eigen::Vector3d> GlobalPlanner::PlanAStarAPF(
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
    const ::voxel_grid_util::VoxelGrid *vg, const double apf_coeff,
    const unsigned int apf_degree, const unsigned int apf_dist,
    const int max_expand, const double weight_heur, const bool verbose) const {

  Eigen::Vector3d start_local = vg->GetCoordLocal(start);
  Eigen::Vector3d goal_local = vg->GetCoordLocal(goal);

  Eigen::Vector3i start_i(start_local[0], start_local[1], start_local[2]);
  Eigen::Vector3i goal_i(goal_local[0], goal_local[1], goal_local[2]);

  GraphSearch graph_search(vg->GetDim(), 1, false);
  graph_search.Plan(vg, start_i, goal_i, false, -1, true, apf_coeff, apf_degree,
                    apf_dist);
  std::vector<StatePtr> path = graph_search.GetPath();
  std::vector<Eigen::Vector3d> path_out =
      ::path_finding_util::ConvertPathToVector(path);

  std::vector<Eigen::Vector3d> path_out_final =
      ::path_finding_util::PathLocalToGlobal(path_out, *vg);

  // reverse the path
  ::std::reverse(path_out_final.begin(), path_out_final.end());

  return path_out;
}

std::vector<Eigen::Vector3d>
GlobalPlanner::PlanJPS(const Eigen::Vector3d &start, const Eigen::Vector3d goal,
                       const ::voxel_grid_util::VoxelGrid *vg,
                       const int max_expand, const double weight_heur,
                       const bool verbose) {
  Eigen::Vector3d start_local = vg->GetCoordLocal(start);
  Eigen::Vector3d goal_local = vg->GetCoordLocal(goal);

  Eigen::Vector3i start_i(start_local[0], start_local[1], start_local[2]);
  Eigen::Vector3i goal_i(goal_local[0], goal_local[1], goal_local[2]);

  GraphSearch graph_search(vg->GetDim(), weight_heur, verbose);
  graph_search.Plan(vg, start_i, goal_i, true);
  std::vector<StatePtr> path = graph_search.GetPath();
  std::vector<Eigen::Vector3d> path_out =
      ::path_finding_util::ConvertPathToVector(path);

  std::vector<Eigen::Vector3d> path_out_final =
      ::path_finding_util::PathLocalToGlobal(path_out, *vg);

  // reverse the path
  ::std::reverse(path_out_final.begin(), path_out_final.end());

  return path_out_final;
}

} // namespace path_finding_util
