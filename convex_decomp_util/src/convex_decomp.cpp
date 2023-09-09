#include "convex_decomp.hpp"

namespace convex_decomp_lib {

Polyhedron3D GetPolyOcta3D(Vec3i seed, std::vector<data_type> &data, Vec3i dim_3D,
                           int n_it, double res_arg, int CONV, Vec3f origin) {
  Border3D borders[6];
  Vec3i faces_list[6];

  for (int i = 0; i < 6; i++) {
    faces_list[i] = seed;
    borders[i].cells.push_back(seed);
  }

  Corner3D corners_list[12];

  Vec3i increments[6] = {Vec3i(0, -1, 0), Vec3i(1, 0, 0), Vec3i(0, 1, 0),
                         Vec3i(-1, 0, 0), Vec3i(0, 0, 1), Vec3i(0, 0, -1)};
  Vec4i adj_corners[6] = {Vec4i(0, 1, 2, 3),   Vec4i(8, 5, 0, 4),
                          Vec4i(10, 9, 8, 11), Vec4i(2, 6, 10, 7),
                          Vec4i(1, 5, 9, 6),   Vec4i(3, 7, 11, 4)};
  Vec4i adj_faces[6] = {Vec4i(1, 4, 3, 5), Vec4i(2, 4, 0, 5),
                        Vec4i(3, 4, 1, 5), Vec4i(0, 4, 2, 5),
                        Vec4i(0, 1, 2, 3), Vec4i(0, 3, 2, 1)};
  Vec4i adj_limits[6] = {Vec4i(2, 0, 0, 0), Vec4i(2, 1, 0, 3),
                         Vec4i(2, 2, 0, 2), Vec4i(2, 3, 0, 1),
                         Vec4i(1, 1, 1, 1), Vec4i(3, 3, 3, 3)};
  Vec2i corner_adj_faces[12] = {Vec2i(0, 1), Vec2i(0, 4), Vec2i(0, 3),
                                Vec2i(0, 5), Vec2i(1, 5), Vec2i(1, 4),
                                Vec2i(3, 4), Vec2i(3, 5), Vec2i(1, 2),
                                Vec2i(2, 4), Vec2i(2, 3), Vec2i(2, 5)};

  std::vector<Vec3i> xy_dir_1 = {Vec3i(1, 0, 0), Vec3i(0, 0, 1)};
  std::vector<Vec3i> xy_dir_2 = {Vec3i(0, 1, 0), Vec3i(0, 0, 1)};
  std::vector<Vec3i> xy_dir_3 = {Vec3i(-1, 0, 0), Vec3i(0, 0, 1)};
  std::vector<Vec3i> xy_dir_4 = {Vec3i(0, -1, 0), Vec3i(0, 0, 1)};
  std::vector<Vec3i> xy_dir_5 = {Vec3i(0, -1, 0), Vec3i(1, 0, 0)};
  std::vector<Vec3i> xy_dir_6 = {Vec3i(0, -1, 0), Vec3i(-1, 0, 0)};

  std::vector<std::vector<Vec3i>> xy_dir = {xy_dir_1, xy_dir_2, xy_dir_3,
                                            xy_dir_4, xy_dir_5, xy_dir_6};

  for (int i = 0; i < 6; i++) {
    borders[i].limits = Vec4i(seed.dot(xy_dir[i][0]), seed.dot(xy_dir[i][1]),
                              -seed.dot(xy_dir[i][0]), -seed.dot(xy_dir[i][1]));
  }

  bool border_validity[6] = {true, true, true, true, true, true};

  int data_idx = seed(0) + seed(1) * int(dim_3D(0)) +
                 seed(2) * int(dim_3D(0)) * int(dim_3D(1));
  data[data_idx] = CONV;

  for (int i = 0; i < n_it; i++) {
    uint8_t idx = (i + 6) % 6;

    if (border_validity[idx]) {
      std::vector<Vec3i> borders_i = borders[idx].cells;
      Vec3i increments_i = increments[idx];
      bool valid_border = true;

      Vec3i x_dir = xy_dir[idx][0];
      Vec3i y_dir = xy_dir[idx][1];

      Vec3i increments_2d[4] = {x_dir, y_dir, -x_dir, -y_dir};

      // find limits
      Vec4i borders_limits(0, 0, 0, 0);
      Corner3D corners_tmp[4];

      for (int j = 0; j < 4; j++) {
        borders_limits(j) = borders[idx].limits(j);
        Corner3D corner = corners_list[adj_corners[idx](j)];
        corners_tmp[j] = corner;
        if (corner.slope > 0) {
          if (corner.fixed) {
            if (corner.direction != idx) {
              if (corner.steps >= corner.slope) {
                borders_limits(j) = borders_limits(j) - 1;
              }
            } else {
              borders_limits(j) = borders_limits(j) - corner.slope;
            }
          } else {
            if (corner.direction == idx) {
              borders_limits(j) = borders_limits(j) - corner.slope;
            }
          }
        }
      }

      // find 2d seed
      bool seed2d_found = false;
      Vec3i seed_2d;

      for (int j = 0; j < int(borders_i.size()); j++) {
        Vec3i cell_tmp = borders_i[j] + increments_i;

        if (cell_tmp(0) >= 1 && cell_tmp(1) >= 1 && cell_tmp(2) >= 1 &&
            cell_tmp(0) < dim_3D(0) - 1 && cell_tmp(1) < dim_3D(1) - 1 &&
            cell_tmp(2) < dim_3D(2) - 1) {
          int data_idx = cell_tmp(0) + cell_tmp(1) * int(dim_3D(0)) +
                         cell_tmp(2) * int(dim_3D(0)) * int(dim_3D(1));
          if (data[data_idx] < CVX_DCMP_OCC &&
              cell_tmp.dot(increments_2d[0]) <= borders_limits(0) &&
              cell_tmp.dot(increments_2d[1]) <= borders_limits(1) &&
              cell_tmp.dot(increments_2d[2]) <= borders_limits(2) &&
              cell_tmp.dot(increments_2d[3]) <= borders_limits(3)) {
            seed_2d = cell_tmp;
            seed2d_found = true;
            break;
          }
        }
      }

      if (seed2d_found) {
        std::deque<Vec3i> seed_vec = {seed_2d};
        std::deque<Vec3i> borders_2d[4] = {seed_vec, seed_vec, seed_vec,
                                           seed_vec};
        std::deque<Vec3i> borders_2d_real[4] = {seed_vec, seed_vec, seed_vec,
                                                seed_vec};

        std::vector<Vec3i> border_real_tmp = {seed_2d};
        Vec3i border_limit_tmp[4] = {seed_2d, seed_2d, seed_2d, seed_2d};
        bool borders_valid_2d[4] = {true, true, true, true};

        int k = 0;
        while (borders_valid_2d[0] || borders_valid_2d[1] ||
               borders_valid_2d[2] || borders_valid_2d[3]) {
          uint8_t idx_2d = (k + 4) % 4;
          uint8_t idx_2d_bef = (k - 1 + 4) % 4;
          uint8_t idx_2d_aft = (k + 1 + 4) % 4;
          k = k + 1;

          std::deque<Vec3i> &borders_2d_i = borders_2d[idx_2d];
          Vec3i increments_2d_i = increments_2d[idx_2d];

          std::deque<Vec3i> border_2d_tmp;
          std::deque<Vec3i> border_tmp;

          bool valid_borders_2d = true;
          for (int j = 0; j < int(borders_2d_i.size()); j++) {
            Vec3i k_idx = borders_2d_i[j] + increments_2d_i;

            if (k_idx.dot(increments_2d_i) <= borders_limits[idx_2d]) {
              Vec3i k_idx_under = k_idx - increments_i;
              int data_idx = k_idx_under(0) + k_idx_under(1) * int(dim_3D(0)) +
                             k_idx_under(2) * int(dim_3D(0)) * int(dim_3D(1));
              if (data[data_idx] == CONV) {
                data_idx = k_idx(0) + k_idx(1) * int(dim_3D(0)) +
                           k_idx(2) * int(dim_3D(0)) * int(dim_3D(1));
                if (data[data_idx] < CVX_DCMP_OCC) {
                  border_2d_tmp.push_back(k_idx);
                  border_tmp.push_back(k_idx);
                } else {
                  valid_borders_2d = false;
                  break;
                }
              } else {
                border_2d_tmp.push_back(k_idx);
              }
            } else {
              valid_borders_2d = false;
              break;
            }
          }

          if (valid_borders_2d) {
            borders_2d[idx_2d] = border_2d_tmp;
            border_real_tmp.insert(border_real_tmp.end(), border_tmp.begin(),
                                   border_tmp.end());
            borders_2d_real[idx_2d] = border_tmp;
            borders_2d[idx_2d_bef].push_back(border_2d_tmp.front());
            borders_2d[idx_2d_aft].push_front(border_2d_tmp.back());

            if (!border_tmp.empty()) {
              Vec3i idx_tmp = border_2d_tmp.front();
              Vec3i idx_real_tmp = border_tmp.front();
              if (idx_tmp(0) == idx_real_tmp(0) &&
                  idx_tmp(1) == idx_real_tmp(1) &&
                  idx_tmp(2) == idx_real_tmp(2)) {
                borders_2d_real[idx_2d_bef].push_back(idx_tmp);
              }
              idx_tmp = border_2d_tmp.back();
              idx_real_tmp = border_tmp.back();
              if (idx_tmp(0) == idx_real_tmp(0) &&
                  idx_tmp(1) == idx_real_tmp(1) &&
                  idx_tmp(2) == idx_real_tmp(2)) {
                borders_2d_real[idx_2d_aft].push_front(idx_tmp);
              }
            }

            for (int j = 0; j < 4; j++) {
              if (!borders_2d_real[j].empty()) {
                border_limit_tmp[j] = borders_2d_real[j].front();
              }
            }
          } else {
            borders_valid_2d[idx_2d] = valid_borders_2d;
          }
        }

        //~ for (int j=0; j<4; j++){
        //~ std::cout << "borders_2d_real: " << j << std::endl;
        //~ for (int j_1=0; j_1<borders_2d_real[j].size(); j_1++){
        //~ std::cout << "borders_2d_real[j].front()" << borders_2d_real[j][j_1]
        //<< std::endl; ~ } ~ }

        for (int j = 0; j < 4; j++) {
          Corner3D corner = corners_tmp[j];
          if (!borders_2d_real[j].empty()) {
            //~ std::cout << "borders[idx].limits(j) " << borders[idx].limits(j)
            //<< std::endl; ~ std::cout << "borders_2d_real[j].front()" <<
            // borders_2d_real[j].front() << std::endl; ~ std::cout <<
            //"increments_2d[j]" << increments_2d[j] << std::endl;
            int dist_max = (borders[idx].limits(j) -
                            borders_2d_real[j].front().dot(increments_2d[j]));
            if (corner.slope == 0) {
              if (dist_max > 0) {
                corner.position =
                    Vec3f(borders_2d_real[j].front()(0) * res_arg -
                              increments_i(0) * res_arg / 2 +
                              increments[adj_faces[idx](j)](0) * res_arg / 2 +
                              res_arg / 2,
                          borders_2d_real[j].front()(1) * res_arg -
                              increments_i(1) * res_arg / 2 +
                              increments[adj_faces[idx](j)](1) * res_arg / 2 +
                              res_arg / 2,
                          borders_2d_real[j].front()(2) * res_arg -
                              increments_i(2) * res_arg / 2 +
                              increments[adj_faces[idx](j)](2) * res_arg / 2 +
                              res_arg / 2);
                corner.slope = dist_max;
                corner.steps = dist_max;
                if (dist_max > 1) {
                  corner.direction = idx;
                }
              }
            } else if (corner.fixed == true) {
              if (corner.direction == idx || corner.direction == -1) {
                if (dist_max > corner.slope) {
                  valid_border = false;
                  break;
                }
              } else {
                if (corner.steps >= corner.slope) {
                  if (dist_max > 1) {
                    valid_border = false;
                    break;
                  } else {
                    corner.steps = 1;
                  }
                } else {
                  if (dist_max != 0) {
                    valid_border = false;
                    break;
                  } else {
                    corner.steps = corner.steps + 1;
                  }
                }
              }
            } else if (corner.fixed == false) {
              if (corner.direction == -1) {
                if (dist_max == 0) {
                  corner.direction = adj_faces[idx](j);
                  corner.steps += 1;
                  corner.slope += 1;
                } else if (dist_max == 1) {
                  corner.fixed = true;
                } else {
                  valid_border = false;
                  break;
                }
              } else if (corner.direction == idx) {
                corner.slope = dist_max;
                corner.fixed = true;
              } else {
                if (dist_max == 0) {
                  corner.slope += 1;
                  corner.steps += 1;
                } else if (dist_max == 1) {
                  corner.fixed = true;
                  corner.steps = 1;
                } else {
                  valid_border = false;
                  break;
                }
              }
            }
          }
          corners_tmp[j] = corner;
        }
        //~ if(i == 52 && CONV == -5){
        //~ std::cout << "here";
        //~ }

        //~ for(int j=0; j<border_real_tmp.size(); j++){
        //~ std::cout << "border_real_tmp" << border_real_tmp[j] << std::endl;
        //~ }
        if (valid_border) {
          borders[idx].cells = border_real_tmp;

          for (int j = 0; j < 4; j++) {
            borders[idx].limits(j) = border_limit_tmp[j].dot(increments_2d[j]);
            corners_list[adj_corners[idx](j)] = corners_tmp[j];
            if (corners_tmp[j].slope == 0 && !borders_2d_real[j].empty()) {
              int dist_max = borders[idx].limits(j) -
                             borders_2d_real[j].front().dot(increments_2d[j]);
              if (dist_max == 0) {
                borders[adj_faces[idx](j)].cells.insert(
                    borders[adj_faces[idx](j)].cells.end(),
                    borders_2d_real[j].begin(), borders_2d_real[j].end());
                borders[adj_faces[idx](j)].limits(adj_limits[idx](j)) += 1;
              }
            }
          }
          faces_list[idx] = border_real_tmp.front();
          //~ std::cout << "newwwwwwwwwwwwwwwwwwwwwwwww borders[idx].cells[j]"
          //<< std::endl;
          for (int j = 0; j < int(border_real_tmp.size()); j++) {
            //~ std::cout << "borders[idx].cells[j]" << borders[idx].cells[j] <<
            // std::endl;
            Vec3i k_idx = border_real_tmp[j];
            int data_idx = k_idx(0) + k_idx(1) * int(dim_3D(0)) +
                           k_idx(2) * int(dim_3D(0)) * int(dim_3D(1));
            data[data_idx] = CONV;
          }
        } else {
          border_validity[idx] = false;
        }
      }
    }
  }

  // create poly
  vec_E<Hyperplane3D> hp_vec;
  for (int j = 0; j < 12; j++) {
    if (corners_list[j].slope > 0) {
      int face_1 = corner_adj_faces[j](0);
      int face_2 = corner_adj_faces[j](1);
      Vec3f normal;
      if (corners_list[j].direction == face_1) {
        normal = Vec3f(corners_list[j].slope * increments[face_1](0) +
                           increments[face_2](0),
                       corners_list[j].slope * increments[face_1](1) +
                           increments[face_2](1),
                       corners_list[j].slope * increments[face_1](2) +
                           increments[face_2](2));
      } else {
        normal = Vec3f(corners_list[j].slope * increments[face_2](0) +
                           increments[face_1](0),
                       corners_list[j].slope * increments[face_2](1) +
                           increments[face_1](1),
                       corners_list[j].slope * increments[face_2](2) +
                           increments[face_1](2));
      }
      hp_vec.push_back(Hyperplane3D(corners_list[j].position + origin, normal));
    }
  }

  for (int j = 0; j < 6; j++) {
    Vec3f point = Vec3f(faces_list[j](0) * res_arg +
                            increments[j](0) * res_arg / 2 + res_arg / 2,
                        faces_list[j](1) * res_arg +
                            increments[j](1) * res_arg / 2 + res_arg / 2,
                        faces_list[j](2) * res_arg +
                            increments[j](2) * res_arg / 2 + res_arg / 2);
    //~ std::cout << "point: " << point << std::endl;
    //~ std::cout << "point + origin: " << point + origin << std::endl;
    Vec3f normal = Vec3f(increments[j](0), increments[j](1), increments[j](2));
    //~ std::cout << "normal: " << normal << std::endl;
    hp_vec.push_back(Hyperplane3D(point + origin, normal));
  }

  return Polyhedron3D(hp_vec);
}

void FindCorners(int idx, Vec3i *increments, bool *border_validity,
                  Border3D *borders, std::vector<std::vector<Vec3i>> &xy_dir,
                  bool &valid_border_final, Corner3D *corners_list,
                  Vec4i *adj_corners, Corner3D *corners_tmp, Vec3i dim_3D,
                  int CONV, std::vector<data_type> &data) {
  if (border_validity[idx]) {
    Vec3i increments_i = increments[idx];
    std::vector<Vec3i> borders_i = borders[idx].cells;

    Vec3i x_dir = xy_dir[idx][0];
    Vec3i y_dir = xy_dir[idx][1];

    Vec3i increments_2d[4] = {x_dir, y_dir, -x_dir, -y_dir};

    // find limits
    Vec4i borders_limits(0, 0, 0, 0);
    // Corner3D corners_tmp[4];

    for (int j = 0; j < 4; j++) {
      borders_limits(j) = borders[idx].limits(j);
      Corner3D corner = corners_list[adj_corners[idx](j)];
      corners_tmp[j] = corner;
      if (corner.slope > 0) {
        if (corner.fixed) {
          if (corner.direction != idx) {
            if (corner.steps >= corner.slope) {
              borders_limits(j) = borders_limits(j) - 1;
            }
          } else {
            borders_limits(j) = borders_limits(j) - corner.slope;
          }
        } else {
          if (corner.direction == idx) {
            borders_limits(j) = borders_limits(j) - corner.slope;
          }
        }
      }
    }

    // compute total area
    double border_area =
        fabs(fabs(borders_limits(0)) - fabs(borders_limits(2))) *
        fabs(fabs(borders_limits(1)) - fabs(borders_limits(3)));

    // find 2d seed
    bool seed2d_found = false;
    Vec3i seed_2d;

    for (int j = 0; j < int(borders_i.size()); j++) {
      Vec3i cell_tmp = borders_i[j] + increments_i;

      if (cell_tmp(0) >= 1 && cell_tmp(1) >= 1 && cell_tmp(2) >= 1 &&
          cell_tmp(0) < dim_3D(0) && cell_tmp(1) < dim_3D(1) &&
          cell_tmp(2) < dim_3D(2)) {
        int data_idx = cell_tmp(0) + cell_tmp(1) * int(dim_3D(0)) +
                       cell_tmp(2) * int(dim_3D(0)) * int(dim_3D(1));
        if (data[data_idx] < CVX_DCMP_OCC &&
            cell_tmp.dot(increments_2d[0]) <= borders_limits(0) &&
            cell_tmp.dot(increments_2d[1]) <= borders_limits(1) &&
            cell_tmp.dot(increments_2d[2]) <= borders_limits(2) &&
            cell_tmp.dot(increments_2d[3]) <= borders_limits(3)) {
          seed_2d = cell_tmp;
          seed2d_found = true;
          break;
        }
      }
    }

    if (seed2d_found) {
      std::deque<Vec3i> seed_vec = {seed_2d};
      std::deque<Vec3i> borders_2d[4] = {seed_vec, seed_vec, seed_vec,
                                         seed_vec};
      std::deque<Vec3i> borders_2d_real[4] = {seed_vec, seed_vec, seed_vec,
                                              seed_vec};

      std::vector<Vec3i> border_real_tmp = {seed_2d};
      Vec3i border_limit_tmp[4] = {seed_2d, seed_2d, seed_2d, seed_2d};
      bool borders_valid_2d[4] = {true, true, true, true};

      int k = 0;
      while (borders_valid_2d[0] || borders_valid_2d[1] ||
             borders_valid_2d[2] || borders_valid_2d[3]) {
        uint8_t idx_2d = (k + 4) % 4;
        uint8_t idx_2d_bef = (k - 1 + 4) % 4;
        uint8_t idx_2d_aft = (k + 1 + 4) % 4;
        k = k + 1;

        std::deque<Vec3i> &borders_2d_i = borders_2d[idx_2d];
        Vec3i increments_2d_i = increments_2d[idx_2d];

        std::deque<Vec3i> border_2d_tmp;
        std::deque<Vec3i> border_tmp;

        bool valid_borders_2d = true;
        for (int j = 0; j < int(borders_2d_i.size()); j++) {
          Vec3i k_idx = borders_2d_i[j] + increments_2d_i;

          if (k_idx.dot(increments_2d_i) <= borders_limits[idx_2d]) {
            Vec3i k_idx_under = k_idx - increments_i;
            int data_idx = k_idx_under(0) + k_idx_under(1) * int(dim_3D(0)) +
                           k_idx_under(2) * int(dim_3D(0)) * int(dim_3D(1));
            if (data[data_idx] == CONV) {
              data_idx = k_idx(0) + k_idx(1) * int(dim_3D(0)) +
                         k_idx(2) * int(dim_3D(0)) * int(dim_3D(1));
              if (data[data_idx] < CVX_DCMP_OCC) {
                border_2d_tmp.push_back(k_idx);
                border_tmp.push_back(k_idx);
              } else {
                valid_borders_2d = false;
                break;
              }
            } else {
              border_2d_tmp.push_back(k_idx);
            }
          } else {
            valid_borders_2d = false;
            break;
          }
        }

        if (valid_borders_2d) {
          borders_2d[idx_2d] = border_2d_tmp;
          border_real_tmp.insert(border_real_tmp.end(), border_tmp.begin(),
                                 border_tmp.end());
          borders_2d_real[idx_2d] = border_tmp;
          borders_2d[idx_2d_bef].push_back(border_2d_tmp.front());
          borders_2d[idx_2d_aft].push_front(border_2d_tmp.back());

          if (!border_tmp.empty()) {
            Vec3i idx_tmp = border_2d_tmp.front();
            Vec3i idx_real_tmp = border_tmp.front();
            if (idx_tmp(0) == idx_real_tmp(0) &&
                idx_tmp(1) == idx_real_tmp(1) &&
                idx_tmp(2) == idx_real_tmp(2)) {
              borders_2d_real[idx_2d_bef].push_back(idx_tmp);
            }
            idx_tmp = border_2d_tmp.back();
            idx_real_tmp = border_tmp.back();
            if (idx_tmp(0) == idx_real_tmp(0) &&
                idx_tmp(1) == idx_real_tmp(1) &&
                idx_tmp(2) == idx_real_tmp(2)) {
              borders_2d_real[idx_2d_aft].push_front(idx_tmp);
            }
          }

          for (int j = 0; j < 4; j++) {
            if (!borders_2d_real[j].empty()) {
              border_limit_tmp[j] = borders_2d_real[j].front();
            }
          }
        } else {
          borders_valid_2d[idx_2d] = valid_borders_2d;
        }
      }

      // compute new border area
      double new_border_area =
          fabs(fabs(borders_2d_real[0].front().dot(increments_2d[0])) -
               fabs(borders_2d_real[2].front().dot(increments_2d[2]))) *
          fabs(fabs(borders_2d_real[1].front().dot(increments_2d[1])) -
               fabs(borders_2d_real[3].front().dot(increments_2d[3])));
      if (new_border_area < border_area / 2) {
        valid_border_final = false;
      }

      for (int j = 0; j < 4; j++) {
        Corner3D corner = corners_tmp[j];
        if (!borders_2d_real[j].empty()) {
          int dist_max = (borders[idx].limits(j) -
                          borders_2d_real[j].front().dot(increments_2d[j]));
          if (corner.slope == 0) {
            if (dist_max > 0) {
              corner.slope = dist_max;
              corner.steps = dist_max;
              if (dist_max > 1) {
                corner.direction = idx;
              }
            }
          }
        }
        corners_tmp[j] = corner;
      }
    }
  } else {
    valid_border_final = false;
  }
}

int GetVoxel(Vec3i coord, Vec3i dim_3D, std::vector<data_type> &data) {
  if (coord(0) >= 0 && coord(1) >= 0 && coord(2) >= 0 && coord(0) < dim_3D(0) &&
      coord(1) < dim_3D(1) && coord(2) < dim_3D(2)) {
    return data[coord(0) + coord(1) * dim_3D(0) +
                coord(2) * dim_3D(0) * dim_3D(1)];
  } else {
    return CVX_DCMP_OCC;
  }
}

bool SideIsEmpty(const std::deque<Vec3i> &borders_2d_real, Vec3i increment_i,
                   Vec3i dim_3D, std::vector<data_type> &data) {
  if (borders_2d_real.size() == 0) {
    return false;
  } else {
    for (auto &k_idx : borders_2d_real) {
      if (GetVoxel(k_idx + increment_i, dim_3D, data) > 0) {
        return false;
      }
    }
    return true;
  }
}

Polyhedron3D GetPolyOcta3DNew(Vec3i seed, std::vector<data_type> &data,
                              Vec3i dim_3D, int n_it, double res_arg, int CONV,
                              Vec3f origin) {
  Border3D borders[6];
  Vec3i faces_list[6];

  for (int i = 0; i < 6; i++) {
    faces_list[i] = seed;
    borders[i].cells.push_back(seed);
  }

  Corner3D corners_list[12];

  Vec3i increments[6] = {Vec3i(0, -1, 0), Vec3i(1, 0, 0), Vec3i(0, 1, 0),
                         Vec3i(-1, 0, 0), Vec3i(0, 0, 1), Vec3i(0, 0, -1)};
  Vec4i adj_corners[6] = {Vec4i(0, 1, 2, 3),   Vec4i(8, 5, 0, 4),
                          Vec4i(10, 9, 8, 11), Vec4i(2, 6, 10, 7),
                          Vec4i(1, 5, 9, 6),   Vec4i(3, 7, 11, 4)};
  Vec4i adj_faces[6] = {Vec4i(1, 4, 3, 5), Vec4i(2, 4, 0, 5),
                        Vec4i(3, 4, 1, 5), Vec4i(0, 4, 2, 5),
                        Vec4i(0, 1, 2, 3), Vec4i(0, 3, 2, 1)};
  Vec4i adj_limits[6] = {Vec4i(2, 0, 0, 0), Vec4i(2, 1, 0, 3),
                         Vec4i(2, 2, 0, 2), Vec4i(2, 3, 0, 1),
                         Vec4i(1, 1, 1, 1), Vec4i(3, 3, 3, 3)};
  Vec2i corner_adj_faces[12] = {Vec2i(0, 1), Vec2i(0, 4), Vec2i(0, 3),
                                Vec2i(0, 5), Vec2i(1, 5), Vec2i(1, 4),
                                Vec2i(3, 4), Vec2i(3, 5), Vec2i(1, 2),
                                Vec2i(2, 4), Vec2i(2, 3), Vec2i(2, 5)};

  std::vector<Vec3i> xy_dir_1 = {Vec3i(1, 0, 0), Vec3i(0, 0, 1)};
  std::vector<Vec3i> xy_dir_2 = {Vec3i(0, 1, 0), Vec3i(0, 0, 1)};
  std::vector<Vec3i> xy_dir_3 = {Vec3i(-1, 0, 0), Vec3i(0, 0, 1)};
  std::vector<Vec3i> xy_dir_4 = {Vec3i(0, -1, 0), Vec3i(0, 0, 1)};
  std::vector<Vec3i> xy_dir_5 = {Vec3i(0, -1, 0), Vec3i(1, 0, 0)};
  std::vector<Vec3i> xy_dir_6 = {Vec3i(0, -1, 0), Vec3i(-1, 0, 0)};

  std::vector<std::vector<Vec3i>> xy_dir = {xy_dir_1, xy_dir_2, xy_dir_3,
                                            xy_dir_4, xy_dir_5, xy_dir_6};

  for (int i = 0; i < 6; i++) {
    borders[i].limits = Vec4i(seed.dot(xy_dir[i][0]), seed.dot(xy_dir[i][1]),
                              -seed.dot(xy_dir[i][0]), -seed.dot(xy_dir[i][1]));
  }

  bool border_validity[6] = {true, true, true, true, true, true};

  int data_idx = seed(0) + seed(1) * int(dim_3D(0)) +
                 seed(2) * int(dim_3D(0)) * int(dim_3D(1));
  data[data_idx] = CONV;

  std::vector<Vec3i> grid;
  grid.push_back(seed);

  for (int i = 0; i < n_it; i++) {
    uint8_t idx = (i + 6) % 6;

    if (border_validity[idx]) {
      std::vector<Vec3i> borders_i = borders[idx].cells;
      Vec3i increments_i = increments[idx];
      bool valid_border = true;

      Vec3i x_dir = xy_dir[idx][0];
      Vec3i y_dir = xy_dir[idx][1];

      Vec3i increments_2d[4] = {x_dir, y_dir, -x_dir, -y_dir};

      // find limits
      Vec4i borders_limits(0, 0, 0, 0);
      Corner3D corners_tmp[4];

      for (int j = 0; j < 4; j++) {
        borders_limits(j) = borders[idx].limits(j);
        Corner3D corner = corners_list[adj_corners[idx](j)];
        corners_tmp[j] = corner;
        if (corner.slope > 0) {
          if (corner.fixed) {
            if (corner.direction != idx) {
              if (corner.steps >= corner.slope) {
                borders_limits(j) = borders_limits(j) - 1;
              }
            } else {
              borders_limits(j) = borders_limits(j) - corner.slope;
            }
          } else {
            if (corner.direction == idx) {
              borders_limits(j) = borders_limits(j) - corner.slope;
            }
          }
        }
      }

      // compute total area
      double border_area =
          fabs(fabs(borders_limits(0)) - fabs(borders_limits(2))) *
          fabs(fabs(borders_limits(1)) - fabs(borders_limits(3)));

      // find 2d seed
      bool seed2d_found = false;
      Vec3i seed_2d;

      for (int j = 0; j < int(borders_i.size()); j++) {
        Vec3i cell_tmp = borders_i[j] + increments_i;

        if (cell_tmp(0) >= 1 && cell_tmp(1) >= 1 && cell_tmp(2) >= 1 &&
            cell_tmp(0) < dim_3D(0) && cell_tmp(1) < dim_3D(1) &&
            cell_tmp(2) < dim_3D(2)) {
          int data_idx = cell_tmp(0) + cell_tmp(1) * int(dim_3D(0)) +
                         cell_tmp(2) * int(dim_3D(0)) * int(dim_3D(1));
          if (data[data_idx] < CVX_DCMP_OCC &&
              cell_tmp.dot(increments_2d[0]) <= borders_limits(0) &&
              cell_tmp.dot(increments_2d[1]) <= borders_limits(1) &&
              cell_tmp.dot(increments_2d[2]) <= borders_limits(2) &&
              cell_tmp.dot(increments_2d[3]) <= borders_limits(3)) {
            seed_2d = cell_tmp;
            seed2d_found = true;
            break;
          }
        }
      }

      if (seed2d_found) {
        std::deque<Vec3i> seed_vec = {seed_2d};
        std::deque<Vec3i> borders_2d[4] = {seed_vec, seed_vec, seed_vec,
                                           seed_vec};
        std::deque<Vec3i> borders_2d_real[4] = {seed_vec, seed_vec, seed_vec,
                                                seed_vec};

        std::vector<Vec3i> border_real_tmp = {seed_2d};
        Vec3i border_limit_tmp[4] = {seed_2d, seed_2d, seed_2d, seed_2d};
        bool borders_valid_2d[4] = {true, true, true, true};

        int k = 0;
        while (borders_valid_2d[0] || borders_valid_2d[1] ||
               borders_valid_2d[2] || borders_valid_2d[3]) {
          uint8_t idx_2d = (k + 4) % 4;
          uint8_t idx_2d_bef = (k - 1 + 4) % 4;
          uint8_t idx_2d_aft = (k + 1 + 4) % 4;
          k = k + 1;

          std::deque<Vec3i> &borders_2d_i = borders_2d[idx_2d];
          Vec3i increments_2d_i = increments_2d[idx_2d];

          std::deque<Vec3i> border_2d_tmp;
          std::deque<Vec3i> border_tmp;

          bool valid_borders_2d = true;
          for (int j = 0; j < int(borders_2d_i.size()); j++) {
            Vec3i k_idx = borders_2d_i[j] + increments_2d_i;
            if (k_idx.dot(increments_2d_i) <= borders_limits[idx_2d]) {
              Vec3i k_idx_under = k_idx - increments_i;
              int data_idx = k_idx_under(0) + k_idx_under(1) * int(dim_3D(0)) +
                             k_idx_under(2) * int(dim_3D(0)) * int(dim_3D(1));
              if (data[data_idx] == CONV) {
                data_idx = k_idx(0) + k_idx(1) * int(dim_3D(0)) +
                           k_idx(2) * int(dim_3D(0)) * int(dim_3D(1));
                if (data[data_idx] < CVX_DCMP_OCC) {
                  border_2d_tmp.push_back(k_idx);
                  border_tmp.push_back(k_idx);
                } else {
                  valid_borders_2d = false;
                  break;
                }
              } else {
                border_2d_tmp.push_back(k_idx);
              }
            } else {
              valid_borders_2d = false;
              break;
            }
          }

          if (valid_borders_2d) {
            borders_2d[idx_2d] = border_2d_tmp;
            border_real_tmp.insert(border_real_tmp.end(), border_tmp.begin(),
                                   border_tmp.end());
            borders_2d_real[idx_2d] = border_tmp;
            borders_2d[idx_2d_bef].push_back(border_2d_tmp.front());
            borders_2d[idx_2d_aft].push_front(border_2d_tmp.back());

            if (!border_tmp.empty()) {
              Vec3i idx_tmp = border_2d_tmp.front();
              Vec3i idx_real_tmp = border_tmp.front();
              if (idx_tmp(0) == idx_real_tmp(0) &&
                  idx_tmp(1) == idx_real_tmp(1) &&
                  idx_tmp(2) == idx_real_tmp(2)) {
                borders_2d_real[idx_2d_bef].push_back(idx_tmp);
              }
              idx_tmp = border_2d_tmp.back();
              idx_real_tmp = border_tmp.back();
              if (idx_tmp(0) == idx_real_tmp(0) &&
                  idx_tmp(1) == idx_real_tmp(1) &&
                  idx_tmp(2) == idx_real_tmp(2)) {
                borders_2d_real[idx_2d_aft].push_front(idx_tmp);
              }
            }

            for (int j = 0; j < 4; j++) {
              if (!borders_2d_real[j].empty()) {
                border_limit_tmp[j] = borders_2d_real[j].front();
              }
            }
          } else {
            borders_valid_2d[idx_2d] = valid_borders_2d;
          }
        }

        //~ for (int j=0; j<4; j++){
        //~ std::cout << "borders_2d_real: " << j << std::endl;
        //~ for (int j_1=0; j_1<borders_2d_real[j].size(); j_1++){
        //~ std::cout << "borders_2d_real[j].front()" <<
        // borders_2d_real[j][j_1]
        //<< std::endl; ~ } ~ }

        // compute new border area
        double new_border_area =
            fabs(fabs(borders_2d_real[0].front().dot(increments_2d[0])) -
                 fabs(borders_2d_real[2].front().dot(increments_2d[2]))) *
            fabs(fabs(borders_2d_real[1].front().dot(increments_2d[1])) -
                 fabs(borders_2d_real[3].front().dot(increments_2d[3])));

        bool soft_valid_border = true;
        if (new_border_area < border_area / 2) {
          soft_valid_border = false;
        }

        int corner_new_state[4] = {0};
        for (int j = 0; j < 4; j++) {
          Corner3D corner = corners_tmp[j];
          if (!borders_2d_real[j].empty()) {
            //~ std::cout << "borders[idx].limits(j) " <<
            // borders[idx].limits(j)
            //<< std::endl; ~ std::cout << "borders_2d_real[j].front()" <<
            // borders_2d_real[j].front() << std::endl; ~ std::cout <<
            //"increments_2d[j]" << increments_2d[j] << std::endl;
            int dist_max = (borders[idx].limits(j) -
                            borders_2d_real[j].front().dot(increments_2d[j]));
            if (corner.slope == 0) {
              if (dist_max > 0) {
                corner.position =
                    Vec3f(borders_2d_real[j].front()(0) * res_arg -
                              increments_i(0) * res_arg / 2 +
                              increments[adj_faces[idx](j)](0) * res_arg / 2 +
                              res_arg / 2 + res_arg / 2,
                          borders_2d_real[j].front()(1) * res_arg -
                              increments_i(1) * res_arg / 2 +
                              increments[adj_faces[idx](j)](1) * res_arg / 2 +
                              res_arg / 2 + res_arg / 2,
                          borders_2d_real[j].front()(2) * res_arg -
                              increments_i(2) * res_arg / 2 +
                              increments[adj_faces[idx](j)](2) * res_arg / 2 +
                              res_arg / 2 + res_arg / 2);
                corner.slope = dist_max;
                corner.steps = dist_max;
                if (dist_max > 1) {
                  corner.direction = idx;

                  // check one side if there is slope
                  corner_new_state[j] = 2;
                } else {
                  // check both sides if there is slope
                  // check first side and second side
                  corner_new_state[j] = 1;
                }

                // check if still starting
                if (i < 6) {
                  soft_valid_border = false;
                }
              }
            } else if (corner.fixed == true) {
              if (corner.direction == idx || corner.direction == -1) {
                if (dist_max > corner.slope) {
                  break;
                }
              } else {
                if (corner.steps >= corner.slope) {
                  if (dist_max > 1) {
                    valid_border = false;
                    break;
                  } else {
                    corner.steps = 1;
                  }
                } else {
                  if (dist_max != 0) {
                    valid_border = false;
                    break;
                  } else {
                    corner.steps = corner.steps + 1;
                  }
                }
              }
            } else if (corner.fixed == false) {
              if (corner.direction == -1) {
                if (dist_max == 0) {
                  corner.direction = adj_faces[idx](j);
                  corner.steps += 1;
                  corner.slope += 1;
                } else if (dist_max == 1) {
                  corner.fixed = true;
                } else {
                  valid_border = false;
                  break;
                }
              } else if (corner.direction == idx) {
                corner.slope = dist_max;
                corner.fixed = true;
              } else {
                if (dist_max == 0) {
                  corner.slope += 1;
                  corner.steps += 1;
                } else if (dist_max == 1) {
                  corner.fixed = true;
                  corner.steps = 1;
                } else {
                  valid_border = false;
                  break;
                }
              }
            }
          }
          corners_tmp[j] = corner;
        }
        //~ if(i == 52 && CONV == -5){
        //~ std::cout << "here";
        //~ }

        //~ for(int j=0; j<border_real_tmp.size(); j++){
        //~ std::cout << "border_real_tmp" << border_real_tmp[j] << std::endl;
        //~ }
        if (valid_border && soft_valid_border) {
          bool valid_expansion = true;
          // check all corners on both sides
          for (int j = 0; j < 4; j++) {
            if (corner_new_state[j] > 0) {
              // std::cout << "corner_new_state[j]: " << j << " "
              // << corner_new_state[j] << std::endl;
              bool first_side = true;
              bool second_side = true;
              // check the expansion side
              first_side =
                  SideIsEmpty(borders_2d_real[j], increments_i, dim_3D, data);
              // std::cout << "borders_2d_real[j]: " <<
              // borders_2d_real[j].size() << std::endl; for (auto k_idx :
              // borders_2d_real[j]) {
              //   std::cout << "k_idx: " << k_idx.transpose() << std::endl;
              // }

              // check the other side too
              int face_idx = adj_faces[idx](j);
              int corner_idx = adj_limits[idx](j);

              std::deque<Vec3i> borders_second_side;
              int limit_tmp = borders[face_idx].limits(corner_idx);
              Vec3i x_dir_tp = xy_dir[face_idx][0];
              Vec3i y_dir_tp = xy_dir[face_idx][1];

              Vec3i increments_2d_tp[4] = {x_dir_tp, y_dir_tp, -x_dir_tp,
                                           -y_dir_tp};
              for (auto k_idx : borders[face_idx].cells) {
                if (k_idx.dot(increments_2d_tp[corner_idx]) == limit_tmp) {
                  borders_second_side.push_back(k_idx);
                }
              }

              // std::cout << "borders_second_side: " <<
              // borders_second_side.size() << std::endl; for (auto k_idx :
              // borders_second_side) {
              //   std::cout << "k_idx: " << k_idx.transpose() << std::endl;
              // }

              if (corner_new_state[j] == 1) {
                second_side = SideIsEmpty(borders_second_side,
                                            increments[face_idx], dim_3D, data);
              }
              valid_expansion = !(first_side && second_side);
              // std::cout << "first_side: " << first_side << std::endl;
              // std::cout << "second_side: " << second_side << std::endl;
              // std::cout << "valid_expansion: " << valid_expansion <<
              // std::endl;
              if (!valid_expansion) {
                border_validity[idx] = false;
                break;
              }
            }
          }

          // check if there is need for further expansion
          if (valid_expansion) {
            if (corner_new_state[0] > 0 || corner_new_state[1] > 0 ||
                corner_new_state[2] > 0 || corner_new_state[3] > 0) {
              // Expand in the same direction
              // first add to convex cells to the grid
              std::vector<int> cell_saved;

              for (int j = 0; j < int(border_real_tmp.size()); j++) {
                Vec3i k_idx = border_real_tmp[j];
                int data_idx = k_idx(0) + k_idx(1) * int(dim_3D(0)) +
                               k_idx(2) * int(dim_3D(0)) * int(dim_3D(1));
                cell_saved.push_back(data[data_idx]);
                data[data_idx] = CONV;
              }

              // modify borders accordingly
              Border3D borders_tmp[6];
              std::copy(std::begin(borders), std::end(borders),
                        std::begin(borders_tmp));
              borders_tmp[idx].cells = border_real_tmp;
              for (int j = 0; j < 4; j++) {
                borders_tmp[idx].limits(j) =
                    border_limit_tmp[j].dot(increments_2d[j]);
              }

              // modify corners accordingly
              Corner3D corners_list_tmp[12];
              std::copy(std::begin(corners_list), std::end(corners_list),
                        std::begin(corners_list_tmp));
              for (int j = 0; j < 4; j++) {
                if (corner_new_state[j] == 0) {
                  corners_list_tmp[adj_corners[idx](j)] = corners_tmp[j];
                }
              }

              // find new corners
              bool valid_border_final = true;
              Corner3D corners_tmp_final[4];

              FindCorners(idx, increments, border_validity, borders_tmp,
                           xy_dir, valid_border_final, corners_list_tmp,
                           adj_corners, corners_tmp_final, dim_3D, CONV, data);

              // Finally restate convex cells from grid
              for (int j = 0; j < int(border_real_tmp.size()); j++) {
                Vec3i k_idx = border_real_tmp[j];
                int data_idx = k_idx(0) + k_idx(1) * int(dim_3D(0)) +
                               k_idx(2) * int(dim_3D(0)) * int(dim_3D(1));
                data[data_idx] = cell_saved[j];
              }

              if (valid_border_final) {
                for (int j = 0; j < 4; j++) {
                  if (corner_new_state[j] == 2) {
                    if (corners_tmp_final[j].slope < corners_tmp[j].slope) {
                      valid_expansion = false;
                      border_validity[idx] = false;
                      break;
                    }
                  }
                }

                // check other side if border is still valid
                if (valid_expansion) {
                  for (int j = 0; j < 4; j++) {
                    if (corner_new_state[j] == 1) {
                      // need to check the other side
                      Corner3D corners_tmp_final_tmp[4];
                      // first find index
                      int face_idx = adj_faces[idx](j);

                      valid_border_final = true;
                      FindCorners(face_idx, increments, border_validity,
                                   borders_tmp, xy_dir, valid_border_final,
                                   corners_list, adj_corners,
                                   corners_tmp_final_tmp, dim_3D, CONV, data);

                      if (valid_border_final) {
                        int corner_idx = adj_limits[idx](j);

                        if ((corners_tmp_final_tmp[corner_idx].slope == 0) &&
                            (corners_tmp_final[j].slope == 0)) {
                          valid_expansion = false;
                          break;
                        }
                      }
                    }
                  }
                }
              }
            }
          }

          if (valid_expansion) {
            borders[idx].cells = border_real_tmp;

            for (int j = 0; j < 4; j++) {
              borders[idx].limits(j) =
                  border_limit_tmp[j].dot(increments_2d[j]);
              corners_list[adj_corners[idx](j)] = corners_tmp[j];
              if (corners_tmp[j].slope == 0 && !borders_2d_real[j].empty()) {
                int dist_max = borders[idx].limits(j) -
                               borders_2d_real[j].front().dot(increments_2d[j]);
                if (dist_max == 0) {
                  borders[adj_faces[idx](j)].cells.insert(
                      borders[adj_faces[idx](j)].cells.end(),
                      borders_2d_real[j].begin(), borders_2d_real[j].end());
                  borders[adj_faces[idx](j)].limits(adj_limits[idx](j)) += 1;
                }
              }
            }
            faces_list[idx] = border_real_tmp.front();
            //~ std::cout << "newwwwwwwwwwwwwwwwwwwwwwwww
            // borders[idx].cells[j]"
            //<< std::endl;
            for (int j = 0; j < int(border_real_tmp.size()); j++) {
              //~ std::cout << "borders[idx].cells[j]" <<
              // borders[idx].cells[j]
              //<<
              // std::endl;
              Vec3i k_idx = border_real_tmp[j];
              int data_idx = k_idx(0) + k_idx(1) * int(dim_3D(0)) +
                             k_idx(2) * int(dim_3D(0)) * int(dim_3D(1));
              data[data_idx] = CONV;
              grid.push_back(k_idx);
            }
          }
        } else {
          if (!valid_border) {
            border_validity[idx] = false;
          }
        }
      }
    }
  }

  // cout the border validity
  // for (int j=0; j<6; j++) {
  //   std::cout << "border_validity[idx]: " << border_validity[j] <<
  //   std::endl;
  // }

  // create poly
  vec_E<Hyperplane3D> hp_vec;
  for (int j = 0; j < 12; j++) {
    if (corners_list[j].slope > 0) {
      int face_1 = corner_adj_faces[j](0);
      int face_2 = corner_adj_faces[j](1);
      Vec3f normal;
      if (corners_list[j].direction == face_1) {
        normal = Vec3f(corners_list[j].slope * increments[face_1](0) +
                           increments[face_2](0),
                       corners_list[j].slope * increments[face_1](1) +
                           increments[face_2](1),
                       corners_list[j].slope * increments[face_1](2) +
                           increments[face_2](2));
      } else {
        normal = Vec3f(corners_list[j].slope * increments[face_2](0) +
                           increments[face_1](0),
                       corners_list[j].slope * increments[face_2](1) +
                           increments[face_1](1),
                       corners_list[j].slope * increments[face_2](2) +
                           increments[face_1](2));
      }
      hp_vec.push_back(Hyperplane3D(corners_list[j].position + origin, normal));
    }
  }

  for (int j = 0; j < 6; j++) {
    Vec3f point = Vec3f(faces_list[j](0) * res_arg +
                            increments[j](0) * res_arg / 2 + res_arg / 2,
                        faces_list[j](1) * res_arg +
                            increments[j](1) * res_arg / 2 + res_arg / 2,
                        faces_list[j](2) * res_arg +
                            increments[j](2) * res_arg / 2 + res_arg / 2);
    //~ std::cout << "point: " << point << std::endl;
    //~ std::cout << "point + origin: " << point + origin << std::endl;
    Vec3f normal = Vec3f(increments[j](0), increments[j](1), increments[j](2));
    //~ std::cout << "normal: " << normal << std::endl;
    hp_vec.push_back(Hyperplane3D(point + origin, normal));
  }

  return Polyhedron3D(hp_vec);
}

} // namespace convex_decomp_lig
