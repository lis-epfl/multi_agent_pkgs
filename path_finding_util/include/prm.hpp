#pragma once

#include "path_tools.hpp"
#include "voxel_grid.hpp"

#include <list>
#include <random>

// maximum number of nodes for getting the shortest path by node number in
// the function SearchPath
#define PATH_FINDING_UTIL_MAX_PATH_NODES 100

namespace path_finding_util {
// Implementation mainly taken from fast_planner:
// https://github.com/HKUST-Aerial-Robotics/Fast-Planner with a lot of
// additional comments for clarity and modifications to answer our needs in
// terms of efficiency and safety (pass by reference, add const, change bad code
// , improve complexity (N² to N*log(N))

/**
 * @brief node of the graph; will be part of a linked list.
 */
class GraphNode {
public:
  // define shared pointer for the GraphNode class; useful for adding neighbors;
  // when the last shared_pointer that points to a node/memory is deleted or
  // points elsewhere, the node is deleted from memory.
  typedef std::shared_ptr<GraphNode> Ptr;

  /**
   * @brief types of nodes
   * @type Guard: a node that is not visible by any other node at the time of
   *              sampling
   *       Connector: a node that is visible by at least 2 guards at the time
   *                  of sampling
   */
  enum NODE_TYPE { Guard = 1, Connector = 2 };

  GraphNode() {}

  /**
   * @brief main constructor
   * @param pos: position of the node
   * @param type: type of the node
   * @param id: id of the node
   */
  GraphNode(Eigen::Vector3d pos, NODE_TYPE type, int id) {
    pos_ = pos;
    type_ = type;
    id_ = id;
  }

  ~GraphNode() {}

  // vector of pointers pointing towards other nodes that are connected to the
  // node i.e. visible to the node
  std::vector<std::shared_ptr<GraphNode>> neighbors_;

  // position of the node in the grid frame (as if the voxel size = 1)
  Eigen::Vector3d pos_;

  // type of the node (connector or guard)
  NODE_TYPE type_;

  // node id
  int id_;
};

class TopologyPRM {
private:
  /*---- functions ----*/

  /**
   * @brief sample a point in the region sample_r_ then bring it to the grid
   *        frame
   * @return sampled point
   */
  inline Eigen::Vector3d GetSample();

  /**
   * @brief find the visibile guards in the nodes of the graph
   * @param pt: the point the we want to find its visible guards
   * @return a vector of shared node pointers to the visible guards
   */
  std::vector<GraphNode::Ptr> FindVisibleGuards(const Eigen::Vector3d &pt);

  /**
   * @brief find if we need to create a connection between 2 visible guards
   * @param g1: guard 1 visible from the connection point
   *        g2: guard 2 visible from the connection point
   *        pt: the connection point that can potentially be added to the graph
   * @return true if no homotopy already exists
   *         false otherwise; if homotopy exists and pt gives a shorter path,
   *         replace the position of the point of the longer path with the
   *         position of pt
   */
  bool NeedConnection(const GraphNode::Ptr g1, const GraphNode::Ptr g2,
                      const Eigen::Vector3d &pt);

  /**
   * @brief do depth first search (DFS) to find the paths to the goal
   */
  void DepthFirstSearch(std::vector<GraphNode::Ptr> &vis);

  /* ---- data members ---- */
  // an unnecessary member (added in the comments above a member) is a member
  // that can be defined directly or passed as a reference to the functions that
  // use it and doesn't need to remain in memory the whole time; it is kept for
  // readability and debugging purposes

  /* data members - non parameters */

  // graph of connected nodes that we will explore to find paths (implemented as
  // a doubly linked list)
  std::list<GraphNode::Ptr> graph_;

  // unnecessary member; the origin of the frame centered in the midpoint
  // between the start and the end
  Eigen::Vector3d translation_;

  // unnecessary member; the rotation that defines a frame whose x_axis is along
  // the direction (end-start), y_axis is the cross(x_axis, [0 0 -1]) and z_axis
  // is the cross(x_axis, y_axis)
  Eigen::Matrix3d rotation_;

  // unnecessary member; region in the frame defined by translation_ and
  // rotation_ in which to sample points for PRM; the sampling region starts at
  // the negative of the values of sample_r_ and ends at the positive of the
  // values i.e. -sample_r_(0) < x < sample_r_(0)
  Eigen::Vector3d sample_r_;

  // unnecessary member; random engine that generates pseudo random values
  std::default_random_engine eng_;

  // unnecessary member; uniform distribution sampler
  std::uniform_real_distribution<double> rand_pos_;

  /* data members - parameters */

  // pointer to a voxel grid for collision checking; preferable to use
  // VoxelGrid::ConstPtr which is a shared_ptr; for now this will do
  const ::voxel_grid_util::VoxelGrid *vg_;

  // size of the sampling region in the frame defined by translation_ and
  // rotation_; first paramater is added to half the distance between the start
  // and end while other parameters indicate directly the size of the sampling
  // space; this sampling space is taken in the negative direction as well
  const Eigen::Vector3d sample_inflate_;

  // maximum sampling time for sampling nodes in the sampling space sample_r_
  const double max_sample_time_;

  // maxiumum sampling num of nodes in the sampling space sample_r_
  const unsigned int max_sample_num_;

  // maximum raycasting distance
  const double max_dist_raycast_;

  // maximum number of raw paths in DFS
  const unsigned int max_raw_path_;

  // maximum number of raw paths for choosing paths with least nodes
  const unsigned int max_raw_path_2_;

  // maximum number of short paths after pruning
  const unsigned int max_short_path_;

  // ratio to the shortest path that allows another path to be considered
  const double ratio_to_short_;

  // raw path that we get from DFS on the pruned graph
  std::vector<std::vector<Eigen::Vector3d>> raw_paths_;

  // short paths that we get from shortening the raw paths
  std::vector<std::vector<Eigen::Vector3d>> short_paths_;

  // filtered paths that we get from keeping only one homotopy path
  std::vector<std::vector<Eigen::Vector3d>> filtered_paths_;

  // final paths that we get from selecting the shortest N paths from the
  // filtered paths
  std::vector<std::vector<Eigen::Vector3d>> final_paths_;

public:
  TopologyPRM();

  /**
   * @brief main constructor
   * @param sample_inflate: inflation dimensions for sampling
   * @param max_sample_time: maximum sampling time
   * @param max_dist_raycast: maximum raycasting distance when checking for
   *        homotopies
   * @param max_raw_path: maximum raw paths to find from depth first search of
   *        the Graph
   * @param max_raw_path_2: maximum raw paths for choosing paths with the least
   *        number of nodes
   * @param max_short_path: maximum number of short paths after shortening the
   *        raw paths
   */
  TopologyPRM(const Eigen::Vector3d &sample_inflate,
              const double max_sample_time, const double max_sample_num,
              const double max_dist_raycast, const unsigned int max_raw_path,
              const unsigned int max_raw_path_2,
              const unsigned int max_short_path, const double ratio_to_short);

  /**
   * @brief create a graph of connected nodes
   * @details that we will explore to find
   *          multiple optimal paths in terms of distance and that are not a
   *          homotopy
   * @param start: staring position in the grid frame
   *         end: end position in the grid frame
   */
  void CreateGraph(const Eigen::Vector3d &start, const Eigen::Vector3d &end);

  /**
   * @brief prune the graph
   * @details prune the graph to remove all the nodes that have less then one
   *          neighbour; after removing the node, rerun the pruning again from
   *          the first node; we exit the function when we traverse all the
   *          nodes in the graph with no node being removed
   */
  void PruneGraph();

  // do depth first search (DFS), sort the path by node number and select paths
  // with less nodes
  void SearchPaths();

  // shortcut the raw paths so that the final distance is minimal
  void ShortenPaths();

  // prune paths that are a homotopy
  void PruneEquivalent();

  // select the shortest N paths
  void SelectShortPaths();

  /**
   * @brief find topological graphs between start and end
   * @details generates graph, prunes it, then generates raw_paths_,
   *          filtered_paths_ and final_paths_
   * @param vg: voxel grid for planning
   *        start: starting point
   *        end: ending point
   */
  void FindTopoPaths(const ::voxel_grid_util::VoxelGrid *vg,
                     const Eigen::Vector3d &start, const Eigen::Vector3d &end);

  // get sample_r_
  Eigen::Vector3d GetSampleR() const;

  // get graph of nodes
  std::list<GraphNode::Ptr> GetGraph() const;

  // get raw path
  std::vector<std::vector<Eigen::Vector3d>> GetRawPaths() const;

  // get short paths
  std::vector<std::vector<Eigen::Vector3d>> GetShortPaths() const;

  // get filtered path
  std::vector<std::vector<Eigen::Vector3d>> GetFilteredPaths() const;

  // get final path
  std::vector<std::vector<Eigen::Vector3d>> GetFinalPaths() const;
};

/**
 * @brief write graph to file
 * @details write to file each node position + neighbor positions on a
 *          row; add 0 after the node position if it's a guard and 1 if it's a
 *          connector; then add all neighbor positions;
 * @param graph: graph to write to the file
 *        file_name: file name
 */
void WriteGraphToFile(const std::list<GraphNode::Ptr> &graph,
                      const std::string &file_name);
} // namespace path_finding_util
