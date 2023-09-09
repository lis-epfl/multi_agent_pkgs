// Graph search header file for JPS
#pragma once

#include "voxel_grid.hpp" // VoxelGrid

#include <Eigen/Dense>
#include <boost/heap/d_ary_heap.hpp> // boost::heap::d_ary_heap
#include <limits>                    // std::numeric_limits
#include <memory>                    // std::shared_ptr
#include <unordered_map>             // std::unordered_map
#include <vector>                    // std::vector

namespace path_finding_util {
// heap element comparison
template <class T> struct compare_state {
  bool operator()(T a1, T a2) const {
    double f1 = a1->g + a1->h;
    double f2 = a2->g + a2->h;
    if ((f1 >= f2 - 0.000001) && (f1 <= f2 + 0.000001))
      return a1->g < a2->g; // if equal compare gvals
    return f1 > f2;
  }
};

// define priority queue
struct State; // forward declaration

// state pointer
using StatePtr = std::shared_ptr<State>;

// priority queue (a binary heap)
using priorityQueue =
    boost::heap::d_ary_heap<StatePtr, boost::heap::mutable_<true>,
                            boost::heap::arity<2>,
                            boost::heap::compare<compare_state<StatePtr>>>;

// node of the graph in graph search
struct State {
  // ID
  int id;

  // coord
  Eigen::Vector3i coord;

  // direction
  Eigen::Vector3i direction;

  // id of predecessors
  int parentId = -1;

  // pointer to heap location
  priorityQueue::handle_type heapkey;

  // g cost
  double g = std::numeric_limits<double>::infinity();
  // heuristic cost
  double h;
  // if has been opened
  bool opened = false;
  // if has been closed
  bool closed = false;

  // 3D constructor
  State(int id, const Eigen::Vector3i &coord, const Eigen::Vector3i &direction)
      : id(id), coord(coord), direction(direction) {}
};

// Search and prune neighbors for JPS 3D
struct JPS3DNeib {
  // for each (dx,dy,dz) these contain:
  //    ns: neighbors that are always added
  //    f1: forced neighbors to check
  //    f2: neighbors to add if f1 is forced
  Eigen::Vector3i ns[27][26];
  Eigen::Vector3i f1[27][12];
  Eigen::Vector3i f2[27][12];
  // nsz contains the number of neighbors for the four different types of moves:
  // no move (norm 0):        26 neighbors always added
  //                          0 forced neighbors to check (never happens)
  //                          0 neighbors to add if forced (never happens)
  // straight (norm 1):       1 neighbor always added
  //                          8 forced neighbors to check
  //                          8 neighbors to add if forced
  // diagonal (norm sqrt(2)): 3 neighbors always added
  //                          8 forced neighbors to check
  //                          12 neighbors to add if forced
  // diagonal (norm sqrt(3)): 7 neighbors always added
  //                          6 forced neighbors to check
  //                          12 neighbors to add if forced
  static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
  JPS3DNeib();

private:
  // create the neighbours that are always added array
  void Neib(const int dx, const int dy, const int dz, const int norm1,
            const int dev, int &tx, int &ty, int &tz);

  // create the neighbours to check and to force arrays
  void FNeib(int dx, int dy, int dz, int norm1, int dev, int &fx, int &fy,
             int &fz, int &nx, int &ny, int &nz);
};

/**
 * @brief GraphSearch class; implement A* and Jump Point Search
 */
class GraphSearch {
public:
  /**
   * @brief 3D graph search constructor
   * @param dim: dimension of the voxel grid
   * @param weight_heur: weight of heuristic, optional, default as 1
   * @param verbose: flag for printing debug info, optional, default as False
   */
  GraphSearch(const Eigen::Vector3i &dim, const double weight_heur = 1,
              const bool verbose = false);

  /**
   * @brief start 3D planning thread
   * @param vg: voxel grid pointer
   * @param start: start vector
   * @param goal: goal vector
   * @param use_jps if true, enable JPS search; else the planner is
   *        implementing A*
   * @param maxExpand maximum number of expansion allowed, optional, default
   *        is -1, means no limitation
   * @param use apf: whether to use artificial potential field to push the path
   *        away from obstacles
   * @param apf_coeff: the coefficent to calculate the cost of a voxel close to
   *        an obstacle (apf_coeff)*1/distance^(apf_degree)
   * @param apf_degree: the degree in the cost function
   *        (apf_coeff)*1/distance^(apf_degree)
   * @param apf_dist: maximal distance (in number of voxels) with which to check
   *        the closest obstacles for adding a cost.
   */
  bool Plan(const ::voxel_grid_util::VoxelGrid *vg,
            const Eigen::Vector3i &start, const Eigen::Vector3i &goal,
            const bool use_jps = false, const int max_expand = -1,
            const bool use_apf = false, const double apf_coeff = 0,
            const unsigned int apf_degree = 1, const unsigned int apf_dist = 2);

  // Get the optimal path
  std::vector<StatePtr> GetPath() const;

  // Get the states in open set
  std::vector<StatePtr> GetOpenSet() const;

  // Get the states in close set
  std::vector<StatePtr> GetCloseSet() const;

  // Get the states in hash map
  std::vector<StatePtr> GetAllSet() const;

private:
  /**
   * @brief a function that finds a path between the start and the goal
   * @param curr_node_ptr: the pointer to the start node
   * @param max_expand: maxium expansion iterations
   * @param start_id: the node id of the start
   * @param goal_id: the node id of the goal
   * @return true if we found a path, false otherwise
   */
  bool Plan(StatePtr &curr_node_ptr, const int max_expand, const int start_id,
            const int goal_id);

  // get successor function for A*
  void GetSucc(const StatePtr &curr, std::vector<int> &succ_ids,
               std::vector<double> &succ_costs);

  // get artificial fields value
  double GetArtificialFieldValue(const Eigen::Vector3i &coord) const;

  // get successor function for JPS
  void GetJpsSucc(const StatePtr &curr, std::vector<int> &succ_ids,
                  std::vector<double> &succ_costs);

  // recover the optimal path
  std::vector<StatePtr> RecoverPath(StatePtr node, int id) const;

  // get subscript
  int CoordToId(const Eigen::Vector3i &coord) const;

  // calculate heuristic
  double GetHeur(const Eigen::Vector3i &coord) const;

  // determine if (x, y, z) has forced neighbor with direction (dx, dy, dz)
  bool HasForced(const Eigen::Vector3i &coord,
                 const Eigen::Vector3i &direction) const;

  // 3D jump; recursive implementation; return true if we find the goal or there
  // is a forced point
  bool Jump(const Eigen::Vector3i &coord, const Eigen::Vector3i &direction,
            Eigen::Vector3i &new_coord, bool &reached_goal) const;

  // check if voxel is free
  bool IsFree(const Eigen::Vector3i &coord) const;

  // check if voxel is occupied
  bool IsOccupied(const Eigen::Vector3i &coord) const;

  // voxel grid that contains obstacle information
  const ::voxel_grid_util::VoxelGrid *vg_;

  // unnecessary member; same info in vg_; dimensions of the grid we are
  // searching.
  Eigen::Vector3i dim_;

  // weight of the heuristic
  const double weight_heur_;

  // use artificial potential fields to push the path away from obstacles by
  // making voxels close to obstacles cost more
  bool use_apf_;

  // apf coefficient
  double apf_coeff_;

  // apf degree
  unsigned int apf_degree_;

  // apf distance in voxels
  unsigned int apf_dist_;

  // if true print diagnostics
  bool verbose_;

  // goal vector
  Eigen::Vector3i goal_;

  // whether to use JPS
  bool use_jps_;

  // priority queue of nodes
  priorityQueue pq_;

  // vector that contains pointers to the nodes (accessed by node id)
  std::vector<StatePtr> hm_;

  // vector that contains whether a node has been seen (accessed by node id)
  std::vector<bool> seen_;

  // final optimal path to return
  std::vector<StatePtr> path_;

  // list of neighbours when expanding a node in A*
  std::vector<Eigen::Vector3i> ns_;

  // a pointer to the struct that allows to get JPS neighbors
  std::shared_ptr<JPS3DNeib> jn3d_;
};

// convert path from state to Vector3i
std::vector<Eigen::Vector3d>
ConvertPathToVector(const std::vector<StatePtr> &path);

// convert multiple paths from state to Vector3i
std::vector<std::vector<Eigen::Vector3d>>
ConvertPathsToVector(const std::vector<std::vector<StatePtr>> &paths);

} // namespace path_finding_util
