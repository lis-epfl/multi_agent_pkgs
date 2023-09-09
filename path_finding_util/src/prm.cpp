#include "prm.hpp"

namespace path_finding_util {

TopologyPRM::TopologyPRM(
    const Eigen::Vector3d &sample_inflate, const double max_sample_time,
    const double max_sample_num, const double max_dist_raycast,
    const unsigned int max_raw_path, const unsigned int max_raw_path_2,
    const unsigned int max_short_path, const double ratio_to_short)
    : sample_inflate_(sample_inflate), max_sample_time_(max_sample_time),
      max_sample_num_(max_sample_num), max_dist_raycast_(max_dist_raycast),
      max_raw_path_(max_raw_path), max_raw_path_2_(max_raw_path_2),
      max_short_path_(max_short_path), ratio_to_short_(ratio_to_short) {
  std::random_device rd;
  eng_ = std::default_random_engine(2);
  rand_pos_ = std::uniform_real_distribution<double>(-1.0, 1.0);
}

inline Eigen::Vector3d TopologyPRM::GetSample() {
  Eigen::Vector3d pt;
  pt(0) = rand_pos_(eng_) * sample_r_(0);
  pt(1) = rand_pos_(eng_) * sample_r_(1);
  pt(2) = rand_pos_(eng_) * sample_r_(2);

  pt = rotation_ * pt + translation_;

  return pt;
}

std::vector<GraphNode::Ptr>
TopologyPRM::FindVisibleGuards(const Eigen::Vector3d &pt) {
  std::vector<GraphNode::Ptr> visib_guards;
  int visib_num = 0;

  // find visible GUARD from pt
  for (std::list<GraphNode::Ptr>::iterator iter = graph_.begin();
       iter != graph_.end(); ++iter) {
    if ((*iter)->type_ == GraphNode::Connector)
      continue;

    if (IsLineClear(pt, (*iter)->pos_, *vg_, max_dist_raycast_)) {
      visib_guards.push_back((*iter));
      ++visib_num;
      if (visib_num > 2)
        break;
    }
  }

  return visib_guards;
}

bool TopologyPRM::NeedConnection(const GraphNode::Ptr g1,
                                 const GraphNode::Ptr g2,
                                 const Eigen::Vector3d &pt) {

  bool verbose = false;
  if (std::fabs(pt(0) - 1.248) < 0.001 || std::fabs(pt(0) - 9.5078) < 0.001) {
    /* verbose = true; */
    /* std::cout << "detected point: " << pt.transpose() << std::endl; */
  }

  std::vector<Eigen::Vector3d> path1(3), path2(3);
  path1[0] = g1->pos_;
  path1[1] = pt;
  path1[2] = g2->pos_;

  path2[0] = g1->pos_;
  path2[2] = g2->pos_;

  if (verbose) {
    std::cout << "guards: " << g1->pos_.transpose() << " "
              << g2->pos_.transpose() << std::endl;
  }
  std::vector<Eigen::Vector3d> connect_pts;
  bool has_connect = false;
  for (int i = 0; i < g1->neighbors_.size(); ++i) {
    for (int j = 0; j < g2->neighbors_.size(); ++j) {
      if (verbose) {
        std::cout << std::endl;
      }
      if (verbose) {
        std::cout << "g1 neigh: " << g1->neighbors_[i]->id_ << " "
                  << g1->neighbors_[i]->pos_.transpose() << std::endl;
        std::cout << "g2 neigh: " << g2->neighbors_[j]->id_ << " "
                  << g2->neighbors_[j]->pos_.transpose() << std::endl;
      }
      if (g1->neighbors_[i]->id_ == g2->neighbors_[j]->id_) {
        path2[1] = g1->neighbors_[i]->pos_;
        if (verbose) {
          std::cout << "neighbor: " << path2[1].transpose() << std::endl;
        }
        bool verbose_homo = false;
        if (g1->neighbors_[i]->id_ == 21 && std::fabs(pt(0) - 1.248) < 0.001)
          verbose_homo = true;
        bool same_topo =
            IsHomotopy(path1, path2, *vg_, -1, max_dist_raycast_, verbose);
        if (same_topo) {
          if (verbose) {
            std::cout << "same homo!" << std::endl;
          }
          // get shorter connection
          if (GetPathLength(path1) < GetPathLength(path2)) {
            g1->neighbors_[i]->pos_ = pt;
            if (verbose) {
              std::cout << "shorter!" << std::endl;
            }
          }
          return false;
        } else {
          if (verbose) {
            std::cout << "different homo!" << std::endl;
          }
        }
      }
    }
  }
  return true;
}

void TopologyPRM::PruneGraph() {
  // prune useless node
  if (graph_.size() > 2) {
    for (std::list<GraphNode::Ptr>::iterator iter1 = graph_.begin();
         iter1 != graph_.end() && graph_.size() > 2; ++iter1) {
      if ((*iter1)->id_ <= 1)
        continue;

      // core
      // std::cout << "id: " << (*iter1)->id_ << std::endl;
      if ((*iter1)->neighbors_.size() <= 1) {
        // delete this node from others' neighbor
        for (std::list<GraphNode::Ptr>::iterator iter2 = graph_.begin();
             iter2 != graph_.end(); ++iter2) {
          for (std::vector<GraphNode::Ptr>::iterator it_nb =
                   (*iter2)->neighbors_.begin();
               it_nb != (*iter2)->neighbors_.end(); ++it_nb) {
            if ((*it_nb)->id_ == (*iter1)->id_) {
              (*iter2)->neighbors_.erase(it_nb);
              break;
            }
          }
        }

        // delete this node from graph, restart checking
        graph_.erase(iter1);
        iter1 = graph_.begin();
      }
    }
  }
}
void TopologyPRM::CreateGraph(const Eigen::Vector3d &start,
                              const Eigen::Vector3d &end) {
  // clear the graph from all nodes
  graph_.clear();

  // create start and end nodes
  GraphNode::Ptr start_node =
      GraphNode::Ptr(new GraphNode(start, GraphNode::Guard, 0));
  GraphNode::Ptr end_node =
      GraphNode::Ptr(new GraphNode(end, GraphNode::Guard, 1));

  // add to graph
  graph_.push_back(start_node);
  graph_.push_back(end_node);

  // sample region
  sample_r_(0) = 0.5 * (end - start).norm() + sample_inflate_(0);
  sample_r_(1) = sample_inflate_(1);
  sample_r_(2) = sample_inflate_(2);

  // transformation
  translation_ = 0.5 * (start + end);

  Eigen::Vector3d xtf, ytf, ztf, downward(0, 0, -1);
  xtf = (end - translation_).normalized();
  ytf = xtf.cross(downward).normalized();
  ztf = xtf.cross(ytf);

  rotation_.col(0) = xtf;
  rotation_.col(1) = ytf;
  rotation_.col(2) = ztf;

  /*---- main loop ----*/

  int node_id = 1;
  unsigned int sample_num = 0;
  double sample_time = 0.0;
  Eigen::Vector3d pt;
  clock_t t_clk;

  while (sample_time < max_sample_time_ && sample_num < max_sample_num_) {
    t_clk = clock();

    pt = GetSample();
    ++sample_num;
    if (vg_->IsOccupied(pt) || !vg_->IsInsideGridInt(pt)) {
      sample_time += (double)(clock() - t_clk) / CLOCKS_PER_SEC;
      continue;
    }

    // find visible guard
    std::vector<GraphNode::Ptr> visib_guards = FindVisibleGuards(pt);
    if (visib_guards.size() == 0) {
      GraphNode::Ptr guard =
          GraphNode::Ptr(new GraphNode(pt, GraphNode::Guard, ++node_id));
      graph_.push_back(guard);
    } else if (visib_guards.size() == 2) {
      // try adding new connection between two guard
      bool need_connect = NeedConnection(visib_guards[0], visib_guards[1], pt);
      if (!need_connect) {
        sample_time += (double)(clock() - t_clk) / CLOCKS_PER_SEC;
        continue;
      }
      // new useful connection needed, add new connector
      GraphNode::Ptr connector =
          GraphNode::Ptr(new GraphNode(pt, GraphNode::Connector, ++node_id));
      graph_.push_back(connector);

      // connect guards
      visib_guards[0]->neighbors_.push_back(connector);
      visib_guards[1]->neighbors_.push_back(connector);

      connector->neighbors_.push_back(visib_guards[0]);
      connector->neighbors_.push_back(visib_guards[1]);
    }

    sample_time += (double)(clock() - t_clk) / CLOCKS_PER_SEC;
  }

  // print the number of samples
  /* std::cout << "[Topo]: sample num: " << sample_num << std::endl; */
}

void TopologyPRM::DepthFirstSearch(std::vector<GraphNode::Ptr> &vis) {
  GraphNode::Ptr cur = vis.back();

  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // check reach goal
    if (cur->neighbors_[i]->id_ == 1) {
      // add this path to paths set
      std::vector<Eigen::Vector3d> path;
      for (int j = 0; j < vis.size(); ++j) {
        path.push_back(vis[j]->pos_);
      }
      path.push_back(cur->neighbors_[i]->pos_);

      raw_paths_.push_back(path);
      if (raw_paths_.size() >= max_raw_path_)
        return;

      break;
    }
  }

  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // skip reach goal;
    if (cur->neighbors_[i]->id_ == 1)
      continue;

    // skip already visited node
    bool revisit = false;
    for (int j = 0; j < vis.size(); ++j) {
      if (cur->neighbors_[i]->id_ == vis[j]->id_) {
        revisit = true;
        break;
      }
    }
    if (revisit)
      continue;

    // recursive search
    vis.push_back(cur->neighbors_[i]);
    DepthFirstSearch(vis);
    if (raw_paths_.size() >= max_raw_path_)
      return;

    vis.pop_back();
  }
}

void TopologyPRM::SearchPaths() {
  raw_paths_.clear();

  std::vector<GraphNode::Ptr> visited;
  visited.push_back(graph_.front());

  DepthFirstSearch(visited);

  // sort the path by node number
  int min_node_num = 100000, max_node_num = 1;
  std::vector<std::vector<int>> path_list(PATH_FINDING_UTIL_MAX_PATH_NODES);
  for (int i = 0; i < raw_paths_.size(); ++i) {
    if (int(raw_paths_[i].size()) > max_node_num)
      max_node_num = raw_paths_[i].size();
    if (int(raw_paths_[i].size()) < min_node_num)
      min_node_num = raw_paths_[i].size();
    path_list[std::min(PATH_FINDING_UTIL_MAX_PATH_NODES - 1,
                       int(raw_paths_[i].size()))]
        .push_back(i);
  }

  // select paths with less nodes
  std::vector<std::vector<Eigen::Vector3d>> filter_raw_paths;

  for (int i = min_node_num; i < PATH_FINDING_UTIL_MAX_PATH_NODES; ++i) {
    bool reach_max = false;
    for (int j = 0; j < path_list[i].size(); ++j) {
      filter_raw_paths.push_back(raw_paths_[path_list[i][j]]);
      if (filter_raw_paths.size() >= max_raw_path_2_) {
        reach_max = true;
        break;
      }
    }
    if (reach_max)
      break;
  }
  /* std::cout << "Raw path num: " << raw_paths_.size() << ", " */
  /*           << filter_raw_paths.size() << std::endl; */

  raw_paths_ = filter_raw_paths;
}

void TopologyPRM::ShortenPaths() {
  short_paths_.resize(raw_paths_.size());
  for (int i = 0; i < raw_paths_.size(); ++i) {
    short_paths_[i] = ShortenPath(raw_paths_[i], *vg_, 1, max_dist_raycast_);
  }
}

void TopologyPRM::PruneEquivalent() {
  filtered_paths_ = PruneEquivalentPaths(short_paths_, *vg_, max_dist_raycast_);
}

void TopologyPRM::SelectShortPaths() {
  final_paths_ = ::path_finding_util::SelectShortPaths(
      filtered_paths_, max_short_path_, ratio_to_short_);
}

void TopologyPRM::FindTopoPaths(const ::voxel_grid_util::VoxelGrid *vg,
                                const Eigen::Vector3d &start,
                                const Eigen::Vector3d &end) {
  clock_t t_clk;

  // set voxel grid
  vg_ = vg;

  // define timing variables
  double graph_time, graph_prune_time, search_time, short_time, path_prune_time,
      select_time;

  /* ---------- create the topo graph ---------- */
  t_clk = clock();
  CreateGraph(start, end);
  graph_time = (double)(clock() - t_clk) / CLOCKS_PER_SEC;

  /* ---------- prune graph ------------*/
  t_clk = clock();
  PruneGraph();
  graph_prune_time = (double)(clock() - t_clk) / CLOCKS_PER_SEC;

  /* ---------- search paths in the graph ---------- */
  t_clk = clock();
  SearchPaths();
  search_time = (double)(clock() - t_clk) / CLOCKS_PER_SEC;

  /* ---------- path shortening ---------- */
  t_clk = clock();
  ShortenPaths();
  short_time = (double)(clock() - t_clk) / CLOCKS_PER_SEC;

  /* ---------- prune equivalent paths ---------- */
  t_clk = clock();
  PruneEquivalent();
  path_prune_time = (double)(clock() - t_clk) / CLOCKS_PER_SEC;

  /* ---------- select N shortest paths ---------- */
  t_clk = clock();
  SelectShortPaths();
  select_time = (double)(clock() - t_clk) / CLOCKS_PER_SEC;

  double total_time = graph_time + graph_prune_time + search_time + short_time +
                      path_prune_time + select_time;

  /* std::cout << std::endl */
  /* << "[Topo]: total time: " << total_time << ", graph: " << graph_time */
  /* << ", graph_prune_time: " << graph_prune_time */
  /* << ", search: " << search_time << ", short: " << short_time */
  /* << ", prune: " << path_prune_time << ", select: " << select_time */
  /* << std::endl; */
}

Eigen::Vector3d TopologyPRM::GetSampleR() const { return sample_r_; }

std::list<GraphNode::Ptr> TopologyPRM::GetGraph() const { return graph_; }

std::vector<std::vector<Eigen::Vector3d>> TopologyPRM::GetRawPaths() const {
  return raw_paths_;
}

std::vector<std::vector<Eigen::Vector3d>> TopologyPRM::GetShortPaths() const {
  return short_paths_;
}

std::vector<std::vector<Eigen::Vector3d>>
TopologyPRM::GetFilteredPaths() const {
  return filtered_paths_;
}

std::vector<std::vector<Eigen::Vector3d>> TopologyPRM::GetFinalPaths() const {
  return final_paths_;
}

void WriteGraphToFile(const std::list<GraphNode::Ptr> &graph,
                      const std::string &file_name) {
  std::ofstream my_file;
  my_file.open(file_name);

  for (std::list<GraphNode::Ptr>::const_iterator iter = graph.begin();
       iter != graph.end(); ++iter) {
    my_file << (*iter)->pos_(0) << ", " << (*iter)->pos_(1) << ", "
            << (*iter)->pos_(2);
    if ((*iter)->type_ == GraphNode::Guard) {
      my_file << ", " << 0;
    } else {
      my_file << ", " << 1;
    }
    for (GraphNode::Ptr neighbor : (*iter)->neighbors_) {
      my_file << ", " << neighbor->pos_(0) << ", " << neighbor->pos_(1) << ", "
              << neighbor->pos_(2);
    }
    my_file << std::endl;
  }
  my_file.close();
}
} // namespace path_finding_util
