// Graph search implementation for JPS
#include "graph_search.hpp"

#include <cmath>

namespace path_finding_util {

GraphSearch::GraphSearch(const Eigen::Vector3i &dim, const double weight_heur,
                         const bool verbose)
    : weight_heur_(weight_heur), verbose_(verbose) {
  dim_ = dim;
  hm_.resize(dim_[0] * dim_[1] * dim_[2]);
  seen_.resize(dim_[0] * dim_[1] * dim_[2], false);

  // set 3D neighbors
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      for (int z = -1; z <= 1; z++) {
        if (x == 0 && y == 0 && z == 0)
          continue;
        ns_.push_back(Eigen::Vector3i(x, y, z));
      }
    }
  }
  jn3d_ = std::make_shared<JPS3DNeib>();
}

inline int GraphSearch::CoordToId(const Eigen::Vector3i &coord) const {
  return coord[0] + coord[1] * dim_[0] + coord[2] * dim_[0] * dim_[1];
}

inline double GraphSearch::GetHeur(const Eigen::Vector3i &pt) const {
  return weight_heur_ * (Eigen::Vector3d(pt(0), pt(1), pt(2)) -
                         Eigen::Vector3d(goal_(0), goal_(1), goal_(2)))
                            .norm();
}

bool GraphSearch::Plan(const ::voxel_grid_util::VoxelGrid *vg,
                       const Eigen::Vector3i &start,
                       const Eigen::Vector3i &goal, const bool use_jps,
                       const int max_expand, const bool use_apf,
                       const double apf_coeff, const unsigned int apf_degree,
                       const unsigned int apf_dist) {
  // set voxel grid
  vg_ = vg;

  // clear priority queue and path
  pq_.clear();
  path_.clear();

  // set artificial potential field variables
  use_apf_ = use_apf;
  apf_coeff_ = apf_coeff;
  apf_degree_ = apf_degree;
  apf_dist_ = apf_dist;

  // set jps
  use_jps_ = use_jps;

  // set goal
  int goal_id = CoordToId(goal);
  goal_ = goal;

  // set start node
  int start_id = CoordToId(start);
  StatePtr currNode_ptr =
      std::make_shared<State>(State(start_id, start, Eigen::Vector3i(0, 0, 0)));
  currNode_ptr->g = 0;
  currNode_ptr->h = GetHeur(start);

  return Plan(currNode_ptr, max_expand, start_id, goal_id);
}

bool GraphSearch::Plan(StatePtr &curr_node_ptr, const int max_expand,
                       const int start_id, const int goal_id) {
  // insert start node
  curr_node_ptr->heapkey = pq_.push(curr_node_ptr);
  curr_node_ptr->opened = true;
  hm_[curr_node_ptr->id] = curr_node_ptr;
  seen_[curr_node_ptr->id] = true;

  int expand_iteration = 0;

  while (true) {
    expand_iteration++;
    // get element with smallest cost
    curr_node_ptr = pq_.top();
    pq_.pop();
    curr_node_ptr->closed = true; // Add to closed list

    if (curr_node_ptr->id == goal_id) {
      path_ = RecoverPath(curr_node_ptr, start_id);
      if (verbose_) {
        printf("Goal Reached!!!!!!\n");
        printf("Expand [%d] nodes!\n", expand_iteration);
      }
    }

    // printf("expand: %d, %d\n", currNode_ptr->x, currNode_ptr->y);
    std::vector<int> succ_ids;
    std::vector<double> succ_costs;

    // get successors
    if (!use_jps_)
      GetSucc(curr_node_ptr, succ_ids, succ_costs);
    else
      GetJpsSucc(curr_node_ptr, succ_ids, succ_costs);

    // process successors
    for (int s = 0; s < (int)succ_ids.size(); s++) {
      // see if we can improve the value of succstate
      StatePtr &child_ptr = hm_[succ_ids[s]];
      double tentative_gval = curr_node_ptr->g + succ_costs[s];

      if (tentative_gval < child_ptr->g - 1e-6) {
        child_ptr->parentId = curr_node_ptr->id; // assign new parent
        child_ptr->g = tentative_gval;           // update gval

        // if currently in OPEN, update
        if (child_ptr->opened && !child_ptr->closed) {
          pq_.increase(child_ptr->heapkey); // update heap
          child_ptr->direction =
              (child_ptr->direction - curr_node_ptr->direction);
          if (child_ptr->direction[0] != 0)
            child_ptr->direction[0] /= std::abs(child_ptr->direction[0]);
          if (child_ptr->direction[1] != 0)
            child_ptr->direction[1] /= std::abs(child_ptr->direction[1]);
          if (child_ptr->direction[2] != 0)
            child_ptr->direction[2] /= std::abs(child_ptr->direction[2]);
        }
        // if currently in CLOSED
        else if (child_ptr->opened && child_ptr->closed) {
          printf("ASTAR ERROR!\n");
        } else // new node, add to heap
        {
          // printf("add to open set: %d, %d\n", child_ptr->x, child_ptr->y);
          child_ptr->heapkey = pq_.push(child_ptr);
          child_ptr->opened = true;
        }
      } //
    }   // process successors

    if (max_expand > 0 && expand_iteration >= max_expand) {
      if (verbose_)
        printf("MaxExpandStep [%d] Reached!!!!!!\n\n", max_expand);
      return false;
    }

    if (pq_.empty()) {
      if (verbose_)
        printf("Priority queue is empty!!!!!!\n\n");
      return false;
    }
  }

  if (verbose_) {
    printf("goal g: %f, h: %f!\n", curr_node_ptr->g, curr_node_ptr->h);
    printf("Expand [%d] nodes!\n", expand_iteration);
  }

  return true;
}

std::vector<StatePtr> GraphSearch::RecoverPath(StatePtr node,
                                               const int start_id) const {
  std::vector<StatePtr> path;
  path.push_back(node);
  while (node && node->id != start_id) {
    node = hm_[node->parentId];
    path.push_back(node);
  }

  return path;
}

void GraphSearch::GetSucc(const StatePtr &curr, std::vector<int> &succ_ids,
                          std::vector<double> &succ_costs) {
  // iterate over neighbours
  for (const Eigen::Vector3i &d : ns_) {
    Eigen::Vector3i new_coord = curr->coord + d;
    if (!IsFree(new_coord))
      continue;

    int new_id = CoordToId(new_coord);
    if (!seen_[new_id]) {
      seen_[new_id] = true;
      hm_[new_id] = std::make_shared<State>(new_id, new_coord, d);
      hm_[new_id]->h = GetHeur(new_coord);
    }

    succ_ids.push_back(new_id);
    double cost_total = Eigen::Vector3d(d(0), d(1), d(2)).norm();
    if (use_apf_) {
      cost_total += GetArtificialFieldValue(new_coord);
    }
    succ_costs.push_back(cost_total);
  }
}

double
GraphSearch::GetArtificialFieldValue(const Eigen::Vector3i &coord) const {
  double cost = 0;
  // search around the coordinate to see if there is an obstacle and compute the
  // distance to that obstacle
  double dist_obs = 1e10;
  bool obs_detected = false;

  for (int i = -(int)apf_dist_; i <= (int)apf_dist_; i++) {
    for (int j = -(int)apf_dist_; j <= (int)apf_dist_; j++) {
      for (int k = -(int)apf_dist_; k <= (int)apf_dist_; k++) {
        Eigen::Vector3i new_coord(coord(0) + i, coord(1) + j, coord(2) + k);
        if (IsOccupied(new_coord)) {
          obs_detected = true;
          double dist_tmp =
              (new_coord(0) - coord(0)) * (new_coord(0) - coord(0)) +
              (new_coord(1) - coord(1)) * (new_coord(1) - coord(1)) +
              (new_coord(2) - coord(2)) * (new_coord(2) - coord(2));
          if (dist_tmp < dist_obs) {
            dist_obs = dist_tmp;
          }
        }
      }
    }
  }
  dist_obs = std::sqrt(dist_obs);

  // compute the cost from the minimum distance to obstacle
  if (obs_detected) {
    cost = apf_coeff_ / std::pow(dist_obs, (double)apf_degree_);
  } else {
    cost = 0;
  }
  return cost;
}

void GraphSearch::GetJpsSucc(const StatePtr &curr, std::vector<int> &succ_ids,
                             std::vector<double> &succ_costs) {

  // get direction norm which will determine the number of neighours to
  // add/check
  const int norm1 = curr->direction.lpNorm<1>();

  // compute direction vector to goal to prioritize exploration directions
  Eigen::Vector3d direction_to_goal = (goal_ - curr->coord).cast<double>();
  direction_to_goal.normalize();

  // get number of neighbours
  int num_neib = jn3d_->nsz[norm1][0];

  // get number of forced neighbours
  int num_fneib = jn3d_->nsz[norm1][1];

  // get the unique ID corresponding to the direction which will allow us to
  // find the corresponding neighours to add and to check if we need to add
  // forced neighbours
  int id = (curr->direction[0] + 1) + 3 * (curr->direction[1] + 1) +
           9 * (curr->direction[2] + 1);

  bool reached_goal = false;
  for (int dev = 0; dev < num_neib + num_fneib; ++dev) {
    Eigen::Vector3i new_coord;
    Eigen::Vector3i direction;

    if (dev < num_neib) {
      // start by checking the directions that are always checked to see if
      // there are neighbours to be added
      direction = jn3d_->ns[id][dev];

      // check if a neighbour in this direction deserves to be added by jumping
      // ahead
      if (!Jump(curr->coord, direction, new_coord, reached_goal))
        continue;
    } else {
      // check the forced neighours and add them if they deserve to be added
      Eigen::Vector3i neigh_coord = curr->coord + jn3d_->f1[id][dev - num_neib];

      // first check if we need to add a potential forced neighour
      if (IsOccupied(neigh_coord)) {
        direction = jn3d_->f2[id][dev - num_neib];

        // check if neighbour deserves to be added by jumping ahead
        if (!Jump(curr->coord, direction, new_coord, reached_goal))
          continue;
      } else
        continue;
    }

    /* std::cout << "new_coord: " << new_coord.transpose() << std::endl; */

    int new_id = CoordToId(new_coord);
    if (!seen_[new_id]) {
      seen_[new_id] = true;
      hm_[new_id] = std::make_shared<State>(new_id, new_coord, direction);
      hm_[new_id]->h = GetHeur(new_coord);
    }

    succ_ids.push_back(new_id);
    succ_costs.push_back(
        (Eigen::Vector3d(new_coord(0), new_coord(1), new_coord(2)) -
         Eigen::Vector3d(curr->coord(0), curr->coord(1), curr->coord(2)))
            .norm());

    if (reached_goal) {
      break;
    }
  }
}

bool GraphSearch::Jump(const Eigen::Vector3i &coord,
                       const Eigen::Vector3i &direction,
                       Eigen::Vector3i &new_coord, bool &reached_goal) const {

  // jump ahead one unit in the corresponding direction
  new_coord = coord + direction;

  // check if we hit a wall/obstacle; if yes, the direction is not interesting
  // and we don't add the node to the priority queue of nodes to explore
  if (!IsFree(new_coord))
    return false;

  // if we hit the goal then the node is interesting and we should add it to the
  // priority queue of nodes to explore
  if (new_coord == goal_) {
    reached_goal = true;
    return true;
  }

  // if the node that we jumped to has a forced neighbour, then we should add it
  // to the queue
  if (HasForced(new_coord, direction))
    return true;

  // if we don't find any cause for stopping (hitting a wall, finding the goal,
  // having a forced neighour) than continue jumping forward
  const int id =
      (direction[0] + 1) + 3 * (direction[1] + 1) + 9 * (direction[2] + 1);
  const int norm1 = direction.lpNorm<1>();
  int num_neib = jn3d_->nsz[norm1][0];

  // jump for all the neighbours except the one in our direction of movement
  // (the one in our direction is the last in our neighbour list that is defined
  // when constructing the object)
  for (int k = 0; k < num_neib - 1; ++k) {
    Eigen::Vector3i new_new_coord;

    // if one of the neighbours has a forced neighbour or reached the goal, we
    // need to add the node to the open set, so return true with the new_coord
    // being our current coordinate
    if (Jump(new_coord, jn3d_->ns[id][k], new_new_coord, reached_goal))
      return true;
  }

  return Jump(new_coord, direction, new_coord, reached_goal);
}

inline bool GraphSearch::HasForced(const Eigen::Vector3i &coord,
                                   const Eigen::Vector3i &direction) const {
  int norm1 = direction.lpNorm<1>();
  int id = (direction[0] + 1) + 3 * (direction[1] + 1) + 9 * (direction[2] + 1);
  switch (norm1) {
  case 1:
    // 1-d move, check 8 neighbors
    for (int fn = 0; fn < 8; ++fn) {
      Eigen::Vector3i neigh_coord = coord + jn3d_->f1[id][fn];
      if (IsOccupied(neigh_coord))
        return true;
    }
    return false;
  case 2:
    // 2-d move, check 8 neighbors
    for (int fn = 0; fn < 8; ++fn) {
      Eigen::Vector3i neigh_coord = coord + jn3d_->f1[id][fn];
      if (IsOccupied(neigh_coord))
        return true;
    }
    return false;
  case 3:
    // 3-d move, check 6 neighbors
    for (int fn = 0; fn < 6; ++fn) {
      Eigen::Vector3i neigh_coord = coord + jn3d_->f1[id][fn];
      if (IsOccupied(neigh_coord))
        return true;
    }
    return false;
  default:
    return false;
  }
}

bool GraphSearch::IsFree(const Eigen::Vector3i &coord) const {
  return vg_->IsFree(coord);
}

bool GraphSearch::IsOccupied(const Eigen::Vector3i &coord) const {
  return vg_->IsOccupied(coord);
}

std::vector<StatePtr> GraphSearch::GetPath() const { return path_; }

std::vector<StatePtr> GraphSearch::GetOpenSet() const {
  std::vector<StatePtr> ss;
  for (const StatePtr &it : hm_) {
    if (it && it->opened && !it->closed)
      ss.push_back(it);
  }
  return ss;
}

std::vector<StatePtr> GraphSearch::GetCloseSet() const {
  std::vector<StatePtr> ss;
  for (const StatePtr &it : hm_) {
    if (it && it->closed)
      ss.push_back(it);
  }
  return ss;
}

std::vector<StatePtr> GraphSearch::GetAllSet() const {
  std::vector<StatePtr> ss;
  for (const StatePtr &it : hm_) {
    if (it)
      ss.push_back(it);
  }
  return ss;
}

constexpr int JPS3DNeib::nsz[4][2];

JPS3DNeib::JPS3DNeib() {
  int id = 0;
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
        int tx, ty, tz;
        for (int dev = 0; dev < nsz[norm1][0]; ++dev) {
          Neib(dx, dy, dz, norm1, dev, tx, ty, tz);
          ns[id][dev] = Eigen::Vector3i(tx, ty, tz);
        }
        int fx, fy, fz, nx, ny, nz;
        for (int dev = 0; dev < nsz[norm1][1]; ++dev) {
          FNeib(dx, dy, dz, norm1, dev, fx, fy, fz, nx, ny, nz);
          f1[id][dev] = Eigen::Vector3i(fx, fy, fz);
          f2[id][dev] = Eigen::Vector3i(nx, ny, nz);
        }
        id++;
      }
    }
  }
}

void JPS3DNeib::Neib(const int dx, const int dy, const int dz, const int norm1,
                     const int dev, int &tx, int &ty, int &tz) {
  switch (norm1) {
  case 0:
    switch (dev) {
    case 0:
      tx = 1;
      ty = 0;
      tz = 0;
      return;
    case 1:
      tx = -1;
      ty = 0;
      tz = 0;
      return;
    case 2:
      tx = 0;
      ty = 1;
      tz = 0;
      return;
    case 3:
      tx = 1;
      ty = 1;
      tz = 0;
      return;
    case 4:
      tx = -1;
      ty = 1;
      tz = 0;
      return;
    case 5:
      tx = 0;
      ty = -1;
      tz = 0;
      return;
    case 6:
      tx = 1;
      ty = -1;
      tz = 0;
      return;
    case 7:
      tx = -1;
      ty = -1;
      tz = 0;
      return;
    case 8:
      tx = 0;
      ty = 0;
      tz = 1;
      return;
    case 9:
      tx = 1;
      ty = 0;
      tz = 1;
      return;
    case 10:
      tx = -1;
      ty = 0;
      tz = 1;
      return;
    case 11:
      tx = 0;
      ty = 1;
      tz = 1;
      return;
    case 12:
      tx = 1;
      ty = 1;
      tz = 1;
      return;
    case 13:
      tx = -1;
      ty = 1;
      tz = 1;
      return;
    case 14:
      tx = 0;
      ty = -1;
      tz = 1;
      return;
    case 15:
      tx = 1;
      ty = -1;
      tz = 1;
      return;
    case 16:
      tx = -1;
      ty = -1;
      tz = 1;
      return;
    case 17:
      tx = 0;
      ty = 0;
      tz = -1;
      return;
    case 18:
      tx = 1;
      ty = 0;
      tz = -1;
      return;
    case 19:
      tx = -1;
      ty = 0;
      tz = -1;
      return;
    case 20:
      tx = 0;
      ty = 1;
      tz = -1;
      return;
    case 21:
      tx = 1;
      ty = 1;
      tz = -1;
      return;
    case 22:
      tx = -1;
      ty = 1;
      tz = -1;
      return;
    case 23:
      tx = 0;
      ty = -1;
      tz = -1;
      return;
    case 24:
      tx = 1;
      ty = -1;
      tz = -1;
      return;
    case 25:
      tx = -1;
      ty = -1;
      tz = -1;
      return;
    }
  case 1:
    tx = dx;
    ty = dy;
    tz = dz;
    return;
  case 2:
    switch (dev) {
    case 0:
      if (dz == 0) {
        tx = 0;
        ty = dy;
        tz = 0;
        return;
      } else {
        tx = 0;
        ty = 0;
        tz = dz;
        return;
      }
    case 1:
      if (dx == 0) {
        tx = 0;
        ty = dy;
        tz = 0;
        return;
      } else {
        tx = dx;
        ty = 0;
        tz = 0;
        return;
      }
    case 2:
      tx = dx;
      ty = dy;
      tz = dz;
      return;
    }
  case 3:
    switch (dev) {
    case 0:
      tx = dx;
      ty = 0;
      tz = 0;
      return;
    case 1:
      tx = 0;
      ty = dy;
      tz = 0;
      return;
    case 2:
      tx = 0;
      ty = 0;
      tz = dz;
      return;
    case 3:
      tx = dx;
      ty = dy;
      tz = 0;
      return;
    case 4:
      tx = dx;
      ty = 0;
      tz = dz;
      return;
    case 5:
      tx = 0;
      ty = dy;
      tz = dz;
      return;
    case 6:
      tx = dx;
      ty = dy;
      tz = dz;
      return;
    }
  }
}

void JPS3DNeib::FNeib(int dx, int dy, int dz, int norm1, int dev, int &fx,
                      int &fy, int &fz, int &nx, int &ny, int &nz) {
  switch (norm1) {
  case 1:
    switch (dev) {
    case 0:
      fx = 0;
      fy = 1;
      fz = 0;
      break;
    case 1:
      fx = 0;
      fy = -1;
      fz = 0;
      break;
    case 2:
      fx = 1;
      fy = 0;
      fz = 0;
      break;
    case 3:
      fx = 1;
      fy = 1;
      fz = 0;
      break;
    case 4:
      fx = 1;
      fy = -1;
      fz = 0;
      break;
    case 5:
      fx = -1;
      fy = 0;
      fz = 0;
      break;
    case 6:
      fx = -1;
      fy = 1;
      fz = 0;
      break;
    case 7:
      fx = -1;
      fy = -1;
      fz = 0;
      break;
    }
    nx = fx;
    ny = fy;
    nz = dz;
    // switch order if different direction
    if (dx != 0) {
      fz = fx;
      fx = 0;
      nz = fz;
      nx = dx;
    }
    if (dy != 0) {
      fz = fy;
      fy = 0;
      nz = fz;
      ny = dy;
    }
    return;
  case 2:
    if (dx == 0) {
      switch (dev) {
      case 0:
        fx = 0;
        fy = 0;
        fz = -dz;
        nx = 0;
        ny = dy;
        nz = -dz;
        return;
      case 1:
        fx = 0;
        fy = -dy;
        fz = 0;
        nx = 0;
        ny = -dy;
        nz = dz;
        return;
      case 2:
        fx = 1;
        fy = 0;
        fz = 0;
        nx = 1;
        ny = dy;
        nz = dz;
        return;
      case 3:
        fx = -1;
        fy = 0;
        fz = 0;
        nx = -1;
        ny = dy;
        nz = dz;
        return;
      case 4:
        fx = 1;
        fy = 0;
        fz = -dz;
        nx = 1;
        ny = dy;
        nz = -dz;
        return;
      case 5:
        fx = 1;
        fy = -dy;
        fz = 0;
        nx = 1;
        ny = -dy;
        nz = dz;
        return;
      case 6:
        fx = -1;
        fy = 0;
        fz = -dz;
        nx = -1;
        ny = dy;
        nz = -dz;
        return;
      case 7:
        fx = -1;
        fy = -dy;
        fz = 0;
        nx = -1;
        ny = -dy;
        nz = dz;
        return;
      // extras
      case 8:
        fx = 1;
        fy = 0;
        fz = 0;
        nx = 1;
        ny = dy;
        nz = 0;
        return;
      case 9:
        fx = 1;
        fy = 0;
        fz = 0;
        nx = 1;
        ny = 0;
        nz = dz;
        return;
      case 10:
        fx = -1;
        fy = 0;
        fz = 0;
        nx = -1;
        ny = dy;
        nz = 0;
        return;
      case 11:
        fx = -1;
        fy = 0;
        fz = 0;
        nx = -1;
        ny = 0;
        nz = dz;
        return;
      }
    } else if (dy == 0) {
      switch (dev) {
      case 0:
        fx = 0;
        fy = 0;
        fz = -dz;
        nx = dx;
        ny = 0;
        nz = -dz;
        return;
      case 1:
        fx = -dx;
        fy = 0;
        fz = 0;
        nx = -dx;
        ny = 0;
        nz = dz;
        return;
      case 2:
        fx = 0;
        fy = 1;
        fz = 0;
        nx = dx;
        ny = 1;
        nz = dz;
        return;
      case 3:
        fx = 0;
        fy = -1;
        fz = 0;
        nx = dx;
        ny = -1;
        nz = dz;
        return;
      case 4:
        fx = 0;
        fy = 1;
        fz = -dz;
        nx = dx;
        ny = 1;
        nz = -dz;
        return;
      case 5:
        fx = -dx;
        fy = 1;
        fz = 0;
        nx = -dx;
        ny = 1;
        nz = dz;
        return;
      case 6:
        fx = 0;
        fy = -1;
        fz = -dz;
        nx = dx;
        ny = -1;
        nz = -dz;
        return;
      case 7:
        fx = -dx;
        fy = -1;
        fz = 0;
        nx = -dx;
        ny = -1;
        nz = dz;
        return;
      // extras
      case 8:
        fx = 0;
        fy = 1;
        fz = 0;
        nx = dx;
        ny = 1;
        nz = 0;
        return;
      case 9:
        fx = 0;
        fy = 1;
        fz = 0;
        nx = 0;
        ny = 1;
        nz = dz;
        return;
      case 10:
        fx = 0;
        fy = -1;
        fz = 0;
        nx = dx;
        ny = -1;
        nz = 0;
        return;
      case 11:
        fx = 0;
        fy = -1;
        fz = 0;
        nx = 0;
        ny = -1;
        nz = dz;
        return;
      }
    } else { // dz==0
      switch (dev) {
      case 0:
        fx = 0;
        fy = -dy;
        fz = 0;
        nx = dx;
        ny = -dy;
        nz = 0;
        return;
      case 1:
        fx = -dx;
        fy = 0;
        fz = 0;
        nx = -dx;
        ny = dy;
        nz = 0;
        return;
      case 2:
        fx = 0;
        fy = 0;
        fz = 1;
        nx = dx;
        ny = dy;
        nz = 1;
        return;
      case 3:
        fx = 0;
        fy = 0;
        fz = -1;
        nx = dx;
        ny = dy;
        nz = -1;
        return;
      case 4:
        fx = 0;
        fy = -dy;
        fz = 1;
        nx = dx;
        ny = -dy;
        nz = 1;
        return;
      case 5:
        fx = -dx;
        fy = 0;
        fz = 1;
        nx = -dx;
        ny = dy;
        nz = 1;
        return;
      case 6:
        fx = 0;
        fy = -dy;
        fz = -1;
        nx = dx;
        ny = -dy;
        nz = -1;
        return;
      case 7:
        fx = -dx;
        fy = 0;
        fz = -1;
        nx = -dx;
        ny = dy;
        nz = -1;
        return;
      // extras
      case 8:
        fx = 0;
        fy = 0;
        fz = 1;
        nx = dx;
        ny = 0;
        nz = 1;
        return;
      case 9:
        fx = 0;
        fy = 0;
        fz = 1;
        nx = 0;
        ny = dy;
        nz = 1;
        return;
      case 10:
        fx = 0;
        fy = 0;
        fz = -1;
        nx = dx;
        ny = 0;
        nz = -1;
        return;
      case 11:
        fx = 0;
        fy = 0;
        fz = -1;
        nx = 0;
        ny = dy;
        nz = -1;
        return;
      }
    }
  case 3:
    switch (dev) {
    case 0:
      fx = -dx;
      fy = 0;
      fz = 0;
      nx = -dx;
      ny = dy;
      nz = dz;
      return;
    case 1:
      fx = 0;
      fy = -dy;
      fz = 0;
      nx = dx;
      ny = -dy;
      nz = dz;
      return;
    case 2:
      fx = 0;
      fy = 0;
      fz = -dz;
      nx = dx;
      ny = dy;
      nz = -dz;
      return;
    // need to check up to here for forced!
    case 3:
      fx = 0;
      fy = -dy;
      fz = -dz;
      nx = dx;
      ny = -dy;
      nz = -dz;
      return;
    case 4:
      fx = -dx;
      fy = 0;
      fz = -dz;
      nx = -dx;
      ny = dy;
      nz = -dz;
      return;
    case 5:
      fx = -dx;
      fy = -dy;
      fz = 0;
      nx = -dx;
      ny = -dy;
      nz = dz;
      return;
    // extras
    case 6:
      fx = -dx;
      fy = 0;
      fz = 0;
      nx = -dx;
      ny = 0;
      nz = dz;
      return;
    case 7:
      fx = -dx;
      fy = 0;
      fz = 0;
      nx = -dx;
      ny = dy;
      nz = 0;
      return;
    case 8:
      fx = 0;
      fy = -dy;
      fz = 0;
      nx = 0;
      ny = -dy;
      nz = dz;
      return;
    case 9:
      fx = 0;
      fy = -dy;
      fz = 0;
      nx = dx;
      ny = -dy;
      nz = 0;
      return;
    case 10:
      fx = 0;
      fy = 0;
      fz = -dz;
      nx = 0;
      ny = dy;
      nz = -dz;
      return;
    case 11:
      fx = 0;
      fy = 0;
      fz = -dz;
      nx = dx;
      ny = 0;
      nz = -dz;
      return;
    }
  }
}

std::vector<Eigen::Vector3d>
ConvertPathToVector(const std::vector<StatePtr> &path) {
  std::vector<Eigen::Vector3d> path_out;
  for (const StatePtr &it : path) {
    path_out.push_back(Eigen::Vector3d(it->coord(0) + 0.5, it->coord(1) + 0.5,
                                       it->coord(2) + 0.5));
  }
  return path_out;
}

std::vector<std::vector<Eigen::Vector3d>>
ConvertPathsToVector(const std::vector<std::vector<StatePtr>> &paths) {
  std::vector<std::vector<Eigen::Vector3d>> paths_out;
  for (const std::vector<StatePtr> &path : paths) {
    paths_out.push_back(ConvertPathToVector(path));
  }
  return paths_out;
}

} // namespace path_finding_util
