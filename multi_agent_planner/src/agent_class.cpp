#include "agent_class.hpp"

namespace multi_agent_planner {
Agent::Agent()
    : ::rclcpp::Node("agent_node"), env_(true), model_(InitializeGurobi(env_)) {
  // declare connection parameters
  DeclareRosParameters();

  // get parameters from file
  InitializeRosParameters();

  // initialize MPC/MIQP parameters
  InitializePlannerParameters();

  // set initial yaw angle to 0
  yaw_ = 0;

  // set up a callback to execute code on shutdown
  on_shutdown(::std::bind(&Agent::OnShutdown, this));

  // initialize trajectory mutexes
  ::std::vector<::std::mutex> mutex_vec(n_rob_);
  traj_other_mtx_.swap(mutex_vec);

  // initialize trajectories of other agents
  traj_other_agents_.resize(n_rob_);

  // initialize communication latency of other agents
  com_latency_ms_.resize(n_rob_);

  // create gurobi model
  CreateGurobiModel();

  // create a subscriber to a topic to receive the current state of the drone:
  // for now we will not do this and assume perfect control (the state of the
  // drone is the second position of the generated trajectory); when receiving
  // the state form a simulator or the real drone, we need to account for delay
  // and time to the next planning iteration by simulating the state forward in
  // time before using it
  // TODO: later

  // create a subscriber to a topic to receive goals:
  // not necessary for now since we are using one goal (taken from ros
  // parameters); we can also generate random goals locally to test reciprocal
  // obstacle avoidance extensively
  // TODO: later

  // create prefix for publishers topic name
  ::std::string topic_name = topic_name_ + "_" + ::std::to_string(id_);

  // create publisher to publish the full generated path
  traj_full_pub_ =
      create_publisher<::multi_agent_planner_msgs::msg::Trajectory>(
          topic_name + "/traj_full", 10);

  // create publisher to publish the generated trajectory
  traj_pub_ = create_publisher<::nav_msgs::msg::Path>(topic_name + "/traj", 10);

  // create publisher to publish the reference trajectory
  traj_ref_pub_ =
      create_publisher<::nav_msgs::msg::Path>(topic_name + "/traj_ref", 10);

  // create publisher to publish the generated path
  path_pub_ = create_publisher<::nav_msgs::msg::Path>(topic_name + "/path", 10);

  // create publisher to publish the traversed trajectory so far
  traj_hist_pub_ =
      create_publisher<::nav_msgs::msg::Path>(topic_name + "/traj_hist", 10);

  // create publisher to publish the polyhedra
  poly_pub_ = create_publisher<::decomp_ros_msgs::msg::PolyhedronArray>(
      topic_name + "/polyhedra", 10);

  // create publisher to publish the seeds of the polyhedra
  seeds_pub_ = create_publisher<::sensor_msgs::msg::PointCloud2>(
      topic_name + "/seeds", 10);

  // create publisher to publish the current position
  pos_pub_ = create_publisher<::visualization_msgs::msg::Marker>(
      topic_name + "/position", 10);
  tf_broadcaster_ = ::std::make_shared<::tf2_ros::TransformBroadcaster>(this);

  // create subscriber vector to other agents
  CreateTrajectorySubsriberVector();

  // initialize voxel grid publishers
  voxel_grid_occ_pub_ = create_publisher<::sensor_msgs::msg::PointCloud2>(
      topic_name + "/voxel_grid_occ", 10);
  voxel_grid_free_pub_ = create_publisher<::sensor_msgs::msg::PointCloud2>(
      topic_name + "/voxel_grid_free", 10);
  voxel_grid_unk_pub_ = create_publisher<::sensor_msgs::msg::PointCloud2>(
      topic_name + "/voxel_grid_unk", 10);

  // initialize voxel grid client
  voxel_grid_client_ = create_client<::env_builder_msgs::srv::GetVoxelGrid>(
      get_grid_service_name_);

  // if use_mapping_util_ create subscriber otherwise create timer to get the
  // voxel grid from env_builder
  if (use_mapping_util_) {
    ::std::string vg_sub_topic =
        "agent_" + ::std::to_string(id_) + "/voxel_grid";
    voxel_grid_sub_ =
        create_subscription<::env_builder_msgs::msg::VoxelGridStamped>(
            vg_sub_topic, 10,
            ::std::bind(&Agent::MappingUtilVoxelGridCallback, this,
                        ::std::placeholders::_1));

  } else {
    voxel_service_timer_ = create_wall_timer(
        ::std::chrono::milliseconds(int(voxel_grid_update_period_ * 1e3)),
        ::std::bind(&Agent::GetVoxelGridAsync, this));
  }

  // create a goal subscriber
  ::std::string goal_sub_topic = "agent_" + ::std::to_string(id_) + "/goal";
  goal_sub_ = create_subscription<::geometry_msgs::msg::PointStamped>(
      goal_sub_topic, 10,
      ::std::bind(&Agent::GoalCallback, this, ::std::placeholders::_1));

  // launch path planning thread
  path_planning_thread_ = ::std::thread(&Agent::UpdatePath, this);

  // launch trajectory planning (MIQP) thread
  traj_planning_thread_ = ::std::thread(&Agent::TrajPlanningIteration, this);
}

void Agent::TrajPlanningIteration() {
  // Set the thread priority to highest
  struct sched_param param;
  param.sched_priority = sched_get_priority_max(SCHED_FIFO);
  int err_code = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (err_code == 0) {
    ::std::cout << "Thread priority set to highest for the "
                   "TrajPlanningIteration thread."
                << ::std::endl;
  } else {
    ::std::cerr << "Failed to set thread priority error code: " << err_code
                << ::std::endl;
  }

  // set trajectory planning period
  double traj_planning_period_ms = step_plan_ * dt_ * 1e3;

  // initialize the current state
  state_curr_ = state_ini_;

  // wait for the planning period until the first path is generated
  while (!path_ready_) {
    if (planner_verbose_) {
      RCLCPP_INFO(get_logger(),
                  "trajectory thread: waiting for path to be ready");
    }
    // publish current position which uses state_curr_
    PublishCurrentPosition();

    // sleep for the planning period
    ::std::this_thread::sleep_for(
        ::std::chrono::milliseconds(int(traj_planning_period_ms)));
  }

  // while ros is running, compute trajectory at a constant frequency
  while (::rclcpp::ok()) {
    // cpu clock start
    clock_t t_start_cpu = clock();

    // wall clock start
    auto t_start_wall = ::std::chrono::high_resolution_clock::now();

    // generate safe corridor
    GenerateSafeCorridor();

    // add safe corridor constraints with other agent constraints
    GenerateTimeAwareSafeCorridor();

    // generate reference trajectory based on the last generated path
    GenerateReferenceTrajectory();

    // solve optimization problem and save the result to member variables
    SolveOptimizationProblem();

    // compute yaw angle to look in the direction of travel
    ComputeYawAngle();

    // if the optimization didnt fail on the first iteration
    if (traj_curr_.size() > 0) {
      // check if we need to increment the reference path
      CheckReferenceTrajIncrement();

      // publish the full generated trajectory for other agents
      PublishTrajectoryFull();
    } else {
      // if at the beginning the optimization failed, do not increment the
      // reference trajectory
      increment_traj_ref_ = false;
    }

    // save cpu computation time
    comp_time_tot_.push_back((double)(clock() - t_start_cpu) / CLOCKS_PER_SEC *
                             1e3);

    /* save wall computation time which includes publishing time */
    // compute wall computation time
    auto t_end_wall = ::std::chrono::high_resolution_clock::now();
    double traj_comp_time_wall_ms =
        ::std::chrono::duration_cast<::std::chrono::nanoseconds>(t_end_wall -
                                                                 t_start_wall)
            .count();
    // convert from nano to milliseconds
    traj_comp_time_wall_ms *= 1e-6;
    // save wall computation time
    comp_time_tot_wall_.push_back(traj_comp_time_wall_ms);

    /* sleep for the remaining time (the remainder of the modulo of the clock
    with the planning period is the sleeping time) */
    // get the clock now
    auto t_wall = ::std::chrono::high_resolution_clock::now();
    // get the time in milliseconds
    long long t_wall_ms =
        ::std::chrono::duration_cast<std::chrono::milliseconds>(
            t_wall.time_since_epoch())
            .count();
    // compute the remaining time by modulo with the planning iteration
    double remaining_sleep_time_ms =
        (dt_ * step_plan_ * 1e3) - t_wall_ms % int(dt_ * step_plan_ * 1e3);

    // sleep thread
    ::std::this_thread::sleep_for(
        ::std::chrono::milliseconds(int(remaining_sleep_time_ms)));

    // update current state according to step_plan_ which indicates how many
    // MPC steps to skip (in case no external simulator)
    t_wall = ::std::chrono::high_resolution_clock::now();
    t_wall_ms = ::std::chrono::duration_cast<std::chrono::milliseconds>(
                    t_wall.time_since_epoch())
                    .count();

    // if the first iteration of the optimization didn't fail
    if (traj_curr_.size() > 0) {
      state_curr_.clear();
      for (int i = 0; i < n_x_; i++) {
        state_curr_.push_back(traj_curr_[step_plan_][i]);
      }
    }

    state_hist_stamp_.push_back(double(t_wall_ms) * 1e-3);

    // save the current state
    state_hist_.push_back(state_curr_);

    /* publish for rviz visualization */
    // publish current position
    PublishCurrentPosition();
    // publish the generated trajectory for rviz visualization
    PublishTrajectory();
    // publish the generated path
    PublishReferencePath();
    // publish the polyhedra seeds
    PublishPolyhedraSeeds();
    // publish the polyhedra
    PublishPolyhedra();
    // publish the traversed trajectory so far
    PublishTrajectoryHistory();
  }
}

void Agent::UpdatePath() {
  // while no voxel grid has been received, wait for the path planning period
  while (!voxel_grid_ready_) {
    if (planner_verbose_) {
      RCLCPP_INFO(get_logger(),
                  "path thread: waiting for voxel grid to be ready");
    }
    ::std::this_thread::sleep_for(
        ::std::chrono::milliseconds(int(10 * path_planning_period_ * 1e3)));
  }

  // while ros is running, plan a path at a constant frequency
  while (::rclcpp::ok()) {
    // cpu clock start
    clock_t t_start_cpu = clock();

    // wall clock start
    auto t_start_wall = ::std::chrono::high_resolution_clock::now();

    // copy voxel grid before modifications
    voxel_grid_mtx_.lock();
    ::voxel_grid_util::VoxelGrid vg_util = voxel_grid_;
    voxel_grid_mtx_.unlock();

    // mutex for the goal variable
    goal_mtx_.lock();
    ::std::vector<double> goal = goal_curr_;
    goal_mtx_.unlock();

    // to be uncommented only when doing the going back and forth of the
    // crazyfly test
    /* int period = int(int(state_hist_.size()) / 135); */
    /* if (period % 2 == 1 && period < 4) { */
    /*   goal[0] = state_ini_[0]; */
    /*   goal[1] = state_ini_[1]; */
    /*   goal[2] = state_ini_[2]; */
    /* } */

    /* if (period > 3){ */
    /*   goal[0] = state_ini_[0]; */
    /*   goal[1] = state_ini_[1]; */
    /*   goal[2] = 0; */
    /* } */

    /* if (period >= 1){ */
    /*   goal[0] = goal_curr_[0]; */
    /*   goal[1] = goal_curr_[1]; */
    /*   goal[2] = 0; */
    /* } */

    // mutex for the reference trajectory to get the start variable
    traj_ref_mtx_.lock();
    ::std::vector<::std::vector<double>> traj_ref_curr = traj_ref_curr_;
    traj_ref_mtx_.unlock();

    // get the start from the current reference trajectory
    ::std::vector<double> start;
    if (reset_path_ && !traj_curr_.empty()) {
      for (int i = 0; i < traj_curr_.size(); i++) {
        auto pt = traj_curr_[i];
        int8_t voxel_val =
            vg_util.GetVoxelGlobal(::Eigen::Vector3d(pt[0], pt[1], pt[2]));
        if (voxel_val != ENV_BUILDER_OCC && voxel_val != ENV_BUILDER_UNK) {
          start = {pt[0], pt[1], pt[2]};
          break;
        }
      }
    } else if (!traj_ref_curr.empty()) {
      // if the MIQP/MPC planner has planned at least once; remove the last
      // points of traj_ref_curr that are not free and then take the last
      // point of the free points as the starting point; not that
      // traj_ref_curr is later used to be concatenated with the generated
      // path so it is essential to remove all the points that are occupied
      ::std::vector<::std::vector<double>> traj_tmp;
      int i = 0;
      while (vg_util.GetVoxelGlobal(
                 ::Eigen::Vector3d(traj_ref_curr[i][0], traj_ref_curr[i][1],
                                   traj_ref_curr[i][2])) != ENV_BUILDER_OCC &&
             vg_util.GetVoxelGlobal(
                 ::Eigen::Vector3d(traj_ref_curr[i][0], traj_ref_curr[i][1],
                                   traj_ref_curr[i][2])) != ENV_BUILDER_UNK &&
             i < traj_ref_points_to_keep_) {
        traj_tmp.push_back(traj_ref_curr[i]);
        i = ::std::min(int(traj_ref_curr.size()) - 1, i + 1);
        if (i == int(traj_ref_curr.size()) - 1)
          break;
      }
      traj_ref_curr = traj_tmp;
      start = {traj_ref_curr.back()[0], traj_ref_curr.back()[1],
               traj_ref_curr.back()[2]};
    } else if (!state_ini_.empty()) {
      // if the planner has not planned the first iteration yet
      start = {state_ini_[0], state_ini_[1], state_ini_[2]};
    }

    // find intermediate goal
    ::std::vector<double> goal_inter = GetIntermediateGoal(goal, vg_util);

    // clear boundary voxels
    ClearBoundary(vg_util);

    // plan a path
    ::std::vector<::std::vector<double>> path_out;
    bool valid_path;
    // plan using original method
    valid_path = GetPath(start, goal_inter, vg_util, path_out);
    // plan using new method
    /* valid_path = GetPathNew(start, goal_inter, vg_util, path_out); */

    if (!valid_path) {
      RCLCPP_ERROR(get_logger(), "Couldn't find valid path using JPS/DMP");
    } else {

      // build the initial path from the start of the reference trajectory to
      // the last point of the reference trajectory if we are not resetting
      // the path (not included in the final path because it is the first
      // point of path_out)
      ::std::vector<::std::vector<double>> path_ini;
      if (traj_ref_curr.size() > 0 && !reset_path_) {
        int path_ini_start_idx = 0;
        int path_ini_end_idx = 0;
        // go through the previous path to see where we should start
        for (int i = 0; i < int(path_curr_.size() - 1); i++) {
          ::std::vector<double> s_1 = path_curr_[i];
          ::std::vector<double> s_2 = path_curr_[i + 1];
          if (IsOnSegment(traj_ref_curr[0], s_1, s_2)) {
            path_ini_start_idx = i + 1;
            break;
          }
        }
        // go through the previous path to see where we should end
        for (int i = 0; i < int(path_curr_.size() - 1); i++) {
          ::std::vector<double> s_1 = path_curr_[i];
          ::std::vector<double> s_2 = path_curr_[i + 1];
          if (IsOnSegment(traj_ref_curr.back(), s_1, s_2)) {
            path_ini_end_idx = i;
            break;
          }
        }
        // build path ini (without the last ref point because it is the start
        // of the path planning algorithm)
        path_ini.push_back(
            {traj_ref_curr[0].begin(), traj_ref_curr[0].begin() + 3});
        for (int i = path_ini_start_idx; i <= path_ini_end_idx; i++) {
          path_ini.push_back(path_curr_[i]);
        }
      }

      // join the reference trajectory to the planned path and update the
      // final path used for planning
      path_mtx_.lock();
      path_curr_.clear();
      path_curr_.insert(path_curr_.begin(), path_ini.begin(), path_ini.end());
      path_curr_.insert(path_curr_.end(), path_out.begin(), path_out.end());
      // remove segments points that give acute angles in the path
      path_curr_ = RemoveZigZagSegments(path_curr_);
      path_mtx_.unlock();

      // set the path_ready_ variable to true so we can start the trajectory
      // planning/optimization
      path_ready_ = true;

      // compute the cpu computation time
      double path_comp_time_cpu_ms =
          (double)(clock() - t_start_cpu) / CLOCKS_PER_SEC * 1e3;
      if (planner_verbose_) {
        RCLCPP_INFO(get_logger(), "path planning time: %.3fms",
                    path_comp_time_cpu_ms);
      }

      // save the computation time of the planner for statistics
      comp_time_path_.push_back(path_comp_time_cpu_ms);

      // publish the generated path
      PublishPath();
    }

    /* sleep for the remaining wall time */
    // compute wall computation time
    auto t_end_wall = ::std::chrono::high_resolution_clock::now();
    double path_comp_time_wall_ms =
        ::std::chrono::duration_cast<::std::chrono::nanoseconds>(t_end_wall -
                                                                 t_start_wall)
            .count();
    // convert from nano to milliseconds
    path_comp_time_wall_ms *= 1e-6;
    // compute remaining time to sleep
    double remaining_sleep_time_ms =
        ::std::max(0.0, path_planning_period_ * 1e3 - path_comp_time_wall_ms);
    // sleep thread
    ::std::this_thread::sleep_for(
        ::std::chrono::milliseconds(int(remaining_sleep_time_ms)));
  }
}

bool Agent::GetPathNew(::std::vector<double> &start_arg,
                       ::std::vector<double> &goal_arg,
                       ::voxel_grid_util::VoxelGrid &voxel_grid,
                       ::std::vector<::std::vector<double>> &path_out) {
  // define boolean to check if path is valid
  bool valid_path = true;

  // define start and goal
  ::Eigen::Vector3d start(start_arg[0], start_arg[1], start_arg[2]);
  ::Eigen::Vector3d goal(goal_arg[0], goal_arg[1], goal_arg[2]);

  // create global planner
  ::path_finding_util::GlobalPlanner global_planner;
  ::std::vector<::Eigen::Vector3d> path_jps =
      global_planner.PlanJPS(start, goal, &voxel_grid);
  path_out = ::path_finding_util::EigenToVector(path_jps);

  // return
  return valid_path;
}

bool Agent::GetPath(::std::vector<double> &start_arg,
                    ::std::vector<double> &goal_arg,
                    ::voxel_grid_util::VoxelGrid &vg_util,
                    ::std::vector<::std::vector<double>> &path_out) {
  // set the namespace
  using namespace JPS;

  // TODO: make the voxel grid so it can also be used here without the need to
  // free unknown voxels and rebuild the potential field free unknown voxels
  /* vg_util.FreeUnknown(); */
  vg_util.SetUnknown(99);

  // define start and goal
  Vec3f start(start_arg[0], start_arg[1], start_arg[2]);
  Vec3f goal(goal_arg[0], goal_arg[1], goal_arg[2]);

  // define clock
  clock_t t_start = clock();

  // store map in map_util
  ::std::shared_ptr<VoxelMapUtil> map_util = ::std::make_shared<VoxelMapUtil>();
  map_util->setMap(vg_util.GetOrigin(), vg_util.GetDim(), vg_util.GetData(),
                   vg_util.GetVoxSize());

  if (planner_verbose_) {
    RCLCPP_INFO(get_logger(), "map_util time: %.3fs",
                (double)(clock() - t_start) / CLOCKS_PER_SEC * 1e3);
  }

  // jps planner
  t_start = clock();
  ::std::unique_ptr<JPSPlanner3D> planner_ptr(
      new JPSPlanner3D(planner_verbose_));
  planner_ptr->setMapUtil(map_util);
  planner_ptr->updateMap();
  bool valid_jps = planner_ptr->plan(start, goal, 1, true);
  if (planner_verbose_) {
    RCLCPP_INFO(get_logger(), "JPS time: %.3fms",
                (double)(clock() - t_start) / CLOCKS_PER_SEC * 1e3);
  }
  auto path_jps = planner_ptr->getRawPath();

  // dmp planner
  // set up dmp planner
  t_start = clock();
  IterativeDMPlanner3D dmp(planner_verbose_);
  dmp.setSearchRadius(Vec3f(dmp_search_rad_, dmp_search_rad_, dmp_search_rad_));
  dmp.setCweight(1);
  dmp.setMap(map_util, start);

  // run dmp planner
  bool valid_dist = dmp.iterativeComputePath(start, goal, path_jps, dmp_n_it_);
  if (planner_verbose_) {
    RCLCPP_INFO(get_logger(), "DMP time: %.3fms",
                (double)(clock() - t_start) / CLOCKS_PER_SEC * 1e3);
  }

  // save dmp planner output
  auto path_dmp = dmp.getRawPath();

  ::std::vector<::std::vector<double>> path_dmp_final;
  // add the start_arg because jps and dmp plan from the centers of the voxels
  // and not from the actual starting point; path shortening takes care of the
  // weird initial segment
  path_dmp_final.push_back(start_arg);
  for (auto pt : path_dmp) {
    ::std::vector<double> path_pt = {pt[0], pt[1], pt[2]};
    path_dmp_final.push_back(path_pt);
  }

  // also add the final goal point because jps and dmp plan to the centers of
  // voxels and not the actual goal; path shortening takes care of the weird
  // final segment
  path_dmp_final.push_back(goal_arg);

  // first check if the path isn't empty or the planning didn't fail
  if (!(valid_jps && valid_dist && path_dmp_final.size() >= 1)) {
    ::std::cout << "valid_jps: " << int(valid_jps)
                << " valid_dmp: " << int(valid_dist)
                << " path dmp size: " << path_dmp_final.size() << ::std::endl;
    return false;
  }

  // shorten the path
  ::std::vector<::std::vector<double>> path_out_final =
      ::path_finding_util::ShortenDMPPath(path_dmp_final, vg_util);
  path_out = path_out_final;

  /* RCLCPP_INFO(get_logger(), "finished planning"); */
  return true;
}

void Agent::CheckReferenceTrajIncrement() {
  // check if the progress is positive and the distance to the trajectory is
  // smaller than a given threshold to increment the reference trajectory
  increment_traj_ref_ = false;
  ::std::vector<double> proj_point;
  double proj_dist;
  double path_progress = ::path_finding_util::GetPathProgress(
      traj_curr_[1], traj_ref_curr_, proj_point, proj_dist);
  /* ::std::cout << "proj point: " << proj_point[0] << " " << proj_point[1] << "
   * " */
  /*             << proj_point[2] << ::std::endl; */
  /* ::std::cout << "proj dist: " << proj_dist << ::std::endl; */
  /* ::std::cout << "path progress: " << path_progress << ::std::endl; */
  if (path_progress > 0 && proj_dist < thresh_dist_) {
    starting_point_ = proj_point;
    increment_traj_ref_ = true;
  }

  /* check if we need to reset path generation in case proj_dist is bigger than
   a certain threshold or the first point of the trajectory is blocked from
   the first point of the reference trajectory */
  // first get voxel grid
  voxel_grid_mtx_.lock();
  ::voxel_grid_util::VoxelGrid vg = voxel_grid_;
  voxel_grid_mtx_.unlock();

  // transform point to voxel coordinates
  ::Eigen::Vector3d traj_1(traj_curr_[1][0], traj_curr_[1][1],
                           traj_curr_[1][2]);
  ::Eigen::Vector3d traj_ref_1(traj_ref_curr_[1][0], traj_ref_curr_[1][1],
                               traj_ref_curr_[1][2]);
  ::Eigen::Vector3d traj_1_local = vg.GetCoordLocal(traj_1);
  ::Eigen::Vector3d traj_ref_1_local = vg.GetCoordLocal(traj_ref_1);
  if (!::path_finding_util::IsLineClear(traj_1_local, traj_ref_1_local, vg, 10,
                                        false)) {
    reset_path_ = true;
  } else {
    reset_path_ = false;
  }
}

void Agent::CreateTrajectorySubsriberVector() {
  // resize the vector to the number of robots
  traj_other_sub_vec_.resize(n_rob_);

  // assign the callback function to each element of the vector with an argument
  // (which is the id of the other agent)
  for (int i = 0; i < n_rob_; i++) {
    if (i != id_) {
      ::std::string topic_name_i = topic_name_ + "_" + ::std::to_string(i);
      traj_other_sub_vec_[i] =
          create_subscription<::multi_agent_planner_msgs::msg::Trajectory>(
              topic_name_i + "/traj_full", 10,
              [this,
               i](const ::multi_agent_planner_msgs::msg::Trajectory::SharedPtr
                      msg) { this->TrajectoryOtherAgentsCallback(msg, i); });
    }
  }
}

void Agent::TrajectoryOtherAgentsCallback(
    const ::multi_agent_planner_msgs::msg::Trajectory::SharedPtr &msg,
    const int &id) {
  // lock mutex and save the trajectory message
  traj_other_mtx_[id].lock();
  traj_other_agents_[id] = *msg;
  traj_other_mtx_[id].unlock();

  // compute the communication time
  auto stamp_other = traj_other_agents_[id].stamp;
  int64_t stamp_ns = stamp_other.sec * 1e9 + stamp_other.nanosec;
  int64_t stamp_now = now().nanoseconds();
  int64_t stamp_diff = stamp_now - stamp_ns;
  com_latency_ms_[id].push_back(stamp_diff / 1e6);
}

void Agent::PublishTrajectoryFull() {
  // create trajectory message
  ::multi_agent_planner_msgs::msg::Trajectory traj_msg;

  // set the time stamp
  traj_msg.stamp = now();

  // set the yaw angle
  traj_msg.yaw = yaw_;

  // create the trajectory message
  for (int i = 0; i <= n_hor_; i++) {
    ::multi_agent_planner_msgs::msg::State state;
    state.position = {traj_curr_[i][0], traj_curr_[i][1], traj_curr_[i][2]};
    state.velocity = {traj_curr_[i][3], traj_curr_[i][4], traj_curr_[i][5]};
    if (n_x_ > 6) {
      state.acceleration = {traj_curr_[i][6], traj_curr_[i][7],
                            traj_curr_[i][8]};
    } else {
      if (i < n_hor_) {
        state.acceleration = {control_curr_[i][0], control_curr_[i][1],
                              control_curr_[i][2]};
      }
    }
    traj_msg.states.push_back(state);
  }

  // set the time step
  traj_msg.dt = dt_;

  // publish the trajectory message
  traj_full_pub_->publish(traj_msg);
}

void Agent::PublishTrajectory() {
  if (traj_curr_.size() > 0) {
    // create path message
    ::nav_msgs::msg::Path traj_msg;

    // set the time stamp and the frame
    traj_msg.header.stamp = now();
    traj_msg.header.frame_id = world_frame_;

    // build the trajectory message
    for (int i = 0; i <= n_hor_; i++) {
      ::geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.pose.position.x = traj_curr_[i][0];
      pose_stamped.pose.position.y = traj_curr_[i][1];
      pose_stamped.pose.position.z = traj_curr_[i][2];
      traj_msg.poses.push_back(pose_stamped);
    }

    // publish the trajectory message
    traj_pub_->publish(traj_msg);
  }
}

void Agent::PublishPath() {
  // create path message
  ::nav_msgs::msg::Path path_msg;

  // get path_curr_ for publishing
  path_mtx_.lock();
  ::std::vector<::std::vector<double>> path_curr = path_curr_;
  path_mtx_.unlock();

  // set the time stamp and the frame
  path_msg.header.stamp = now();
  path_msg.header.frame_id = world_frame_;

  // build the trajectory message
  for (int i = 0; i < int(path_curr.size()); i++) {
    ::geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = path_curr[i][0];
    pose_stamped.pose.position.y = path_curr[i][1];
    pose_stamped.pose.position.z = path_curr[i][2];
    path_msg.poses.push_back(pose_stamped);
  }

  // publish the trajectory message
  path_pub_->publish(path_msg);
}

void Agent::PublishTrajectoryHistory() {
  // create path message
  ::nav_msgs::msg::Path path_msg;

  // set the time stamp and the frame
  path_msg.header.stamp = now();
  path_msg.header.frame_id = world_frame_;

  // build the trajectory message
  for (int i = 0; i < int(state_hist_.size() - 1); i++) {
    ::geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = state_hist_[i][0];
    pose_stamped.pose.position.y = state_hist_[i][1];
    pose_stamped.pose.position.z = state_hist_[i][2];
    path_msg.poses.push_back(pose_stamped);
  }

  // publish the trajectory message
  traj_hist_pub_->publish(path_msg);
}

void Agent::PublishReferencePath() {
  // create reference trajectory message
  ::nav_msgs::msg::Path traj_ref_msg;

  // get path_curr_ for publishing
  traj_ref_mtx_.lock();
  ::std::vector<::std::vector<double>> traj_ref_curr = traj_ref_curr_;
  traj_ref_mtx_.unlock();

  // set the time stamp and the frame
  traj_ref_msg.header.stamp = now();
  traj_ref_msg.header.frame_id = world_frame_;

  // build the trajectory message
  for (int i = 0; i < int(traj_ref_curr.size()); i++) {
    ::geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = traj_ref_curr[i][0];
    pose_stamped.pose.position.y = traj_ref_curr[i][1];
    pose_stamped.pose.position.z = traj_ref_curr[i][2];
    traj_ref_msg.poses.push_back(pose_stamped);
  }

  // publish the trajectory message
  traj_ref_pub_->publish(traj_ref_msg);
}

void Agent::PublishCurrentPosition() {
  ::std::vector<double> state_curr;
  // if we started the trajectory generation, use it for state_curr
  if (traj_curr_.size() > 0) {
    state_curr = traj_curr_[0];
  } else {
    state_curr = state_curr_;
  }

  ::std::string tf_name = "agent_" + ::std::to_string(id_);
  // create transform
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = world_frame_;
  transform.child_frame_id = tf_name;

  // Set the translation (position) of the transform
  transform.transform.translation.x = state_curr[0];
  transform.transform.translation.y = state_curr[1];
  transform.transform.translation.z = state_curr[2];

  // compute attitude
  ::Eigen::Quaterniond quat = ComputeAttitude();

  // Set the rotation (orientation) of the transform
  transform.transform.rotation.x = quat.x();
  transform.transform.rotation.y = quat.y();
  transform.transform.rotation.z = quat.z();
  transform.transform.rotation.w = quat.w();

  // Publish the transform
  tf_broadcaster_->sendTransform(transform);

  // create marker
  ::visualization_msgs::msg::Marker marker_msg;
  marker_msg.header.frame_id = world_frame_;
  marker_msg.header.stamp = now();
  marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.pose.position.x = state_curr[0];
  marker_msg.pose.position.y = state_curr[1];
  marker_msg.pose.position.z = state_curr[2];
  marker_msg.pose.orientation.w = 1.0;
  marker_msg.scale.x = 2 * drone_radius_;
  marker_msg.scale.y = 2 * drone_radius_;
  marker_msg.scale.z = 2 * drone_z_offset_;
  marker_msg.color.a = 1;
  marker_msg.color.r = 0.0;
  marker_msg.color.g = 1.0;
  marker_msg.color.b = 0.0;

  // publish marker
  pos_pub_->publish(marker_msg);
}

void Agent::PublishPolyhedraSeeds() {
  // define pointcloud that contains seeds
  ::pcl::PointCloud<::pcl::PointXYZ> seed_cloud;

  // add seed points to pointcloud
  for (int i = 0; i < int(poly_seeds_.size()); i++) {
    ::pcl::PointXYZ pt;
    pt.x = poly_seeds_[i][0];
    pt.y = poly_seeds_[i][1];
    pt.z = poly_seeds_[i][2];
    seed_cloud.push_back(pt);
  }

  // create pointcloud message and publish it
  auto seed_cloud_msg = ::std::make_shared<::sensor_msgs::msg::PointCloud2>();
  ::pcl::toROSMsg(seed_cloud, *seed_cloud_msg);
  seed_cloud_msg->header.frame_id = world_frame_;
  seeds_pub_->publish(*seed_cloud_msg);
}

void Agent::PublishPolyhedra() {
  // create polyhedra message
  ::decomp_ros_msgs::msg::PolyhedronArray poly_msg =
      ::DecompROS::polyhedron_array_to_ros(poly_vec_);
  poly_msg.header.frame_id = world_frame_;
  poly_pub_->publish(poly_msg);
}

void Agent::SolveOptimizationProblem() {
  // start clock
  clock_t t_start = clock();

  // get the reference trajectory (no need for a mutex here but we add it for
  // future proofing)
  traj_ref_mtx_.lock();
  ::std::vector<::std::vector<double>> traj_ref_curr = traj_ref_curr_;
  traj_ref_mtx_.unlock();

  // define the objective function to take into account the reference trajectory
  // which is of size N and does not include the first point of the trajectory
  GRBQuadExpr obj_i(obj_grb_);
  for (int i = 1; i <= n_hor_; i++) {
    if (i == n_hor_) {
      for (int j = 0; j < 6; j++) {
        obj_i += r_n_[j] * (x_grb_[i][j] - traj_ref_curr[i - 1][j]) *
                 (x_grb_[i][j] - traj_ref_curr[i - 1][j]);
      }
    } else {
      for (int j = 0; j < 6; j++) {
        obj_i += r_x_[j] * (x_grb_[i][j] - traj_ref_curr[i - 1][j]) *
                 (x_grb_[i][j] - traj_ref_curr[i - 1][j]);
      }
    }
  }

  // set the first discrete point to the current state
  for (int i = 0; i < n_x_; i++) {
    x_grb_[0][i].set(GRB_DoubleAttr_LB, state_curr_[i]);
    x_grb_[0][i].set(GRB_DoubleAttr_UB, state_curr_[i]);
  }

  // remove previous polyhedra constraints
  if (at_least_1_poly_constr_grb_.size() > 0) {
    for (int i = 0; i < int(at_least_1_poly_constr_grb_.size()); i++) {
      model_.remove(at_least_1_poly_constr_grb_[i]);
    }
    for (int i = 0; i < int(poly_constr_grb_.size()); i++) {
      model_.remove(poly_constr_grb_[i]);
    }
    at_least_1_poly_constr_grb_.clear();
    poly_constr_grb_.clear();
  }

  // set the new polyhedra constraints
  ::std::vector<LinearConstraint3D> polyhedra;
  GRBLinExpr sum_bin;
  ::std::vector<GRBLinExpr> const_i;
  ::std::vector<GRBLinExpr> const_i_1;

  for (int i = 0; i < n_hor_; i++) {
    polyhedra = poly_const_final_vec_[i];
    sum_bin = 0;

    for (int j = 0; j < ::std::min(poly_hor_, int(polyhedra.size())); j++) {
      sum_bin = sum_bin + b_grb_[i][j];

      GRBLinExpr x_expr[n_x_];
      for (int j = 0; j < n_x_; j++) {
        x_expr[j] = GRBLinExpr(x_grb_[i][j]);
      }
      const_i = GetGurobiPolyhedronConstraints(polyhedra.at(j), x_expr);

      for (int j = 0; j < n_x_; j++) {
        x_expr[j] = GRBLinExpr(x_grb_[i + 1][j]);
      }
      const_i_1 = GetGurobiPolyhedronConstraints(polyhedra.at(j), x_expr);

      for (int l = 0; l < int(const_i.size()); l++) {
        poly_constr_grb_.push_back(model_.addGenConstrIndicator(
            b_grb_[i][j], 1, const_i[l], GRB_LESS_EQUAL, 0,
            "poly_const" + ::std::to_string(i) + ::std::to_string(j) +
                ::std::to_string(l)));
        poly_constr_grb_.push_back(model_.addGenConstrIndicator(
            b_grb_[i][j], 1, const_i_1[l], GRB_LESS_EQUAL, 0,
            "poly_const_1" + ::std::to_string(i) + ::std::to_string(j) +
                ::std::to_string(l)));
      }
    }

    at_least_1_poly_constr_grb_.push_back(model_.addConstr(
        sum_bin == 1, "at_least_1_poly_" + ::std::to_string(i)));
  }

  // add the objective function to the model
  model_.setObjective(obj_i, GRB_MINIMIZE);

  // set number of threads to 1 because it results in lower computation time
  // from experiments
  model_.set("Threads", ::std::to_string(1));

  // set the maximum solving time in order to avoid going over the time
  // step/period of the planning
  model_.set(GRB_DoubleParam_TimeLimit, 0.08);

  // optimize model
  optimization_failed_ = false;
  ::std::vector<::std::vector<double>> traj_curr = traj_curr_;
  ::std::vector<::std::vector<double>> control_curr = control_curr_;
  try {
    model_.optimize();
    /* save the result of the optimization to the member variables */
    // first clear previous results
    traj_curr_.clear();
    control_curr_.clear();
    // go through each variable and add it to the traj_curr_/control_curr_
    // also check which polyhedron it belongs to keep it for next iteration
    poly_used_idx_.clear();
    poly_used_idx_.resize(poly_hor_, false);
    for (int i = 0; i < n_hor_ + 1; i++) {
      ::std::vector<double> state;
      for (int j = 0; j < n_x_; j++) {
        state.push_back(x_grb_[i][j].get(GRB_DoubleAttr_X));
      }
      traj_curr_.push_back(state);
      // if we aren't at the last step, save the control input
      if (i < n_hor_) {
        control_curr_.push_back({u_grb_[i][0].get(GRB_DoubleAttr_X),
                                 u_grb_[i][1].get(GRB_DoubleAttr_X),
                                 u_grb_[i][2].get(GRB_DoubleAttr_X)});

        // check which polyhedron the point belongs to
        for (int j = 0; j < poly_hor_; j++) {
          if (b_grb_[i][j].get(GRB_DoubleAttr_X)) {
            poly_used_idx_[j] = true;
          }
        }
      }
    }
  } catch (GRBException e) {
    optimization_failed_ = true;
    ::std::cout << "Error code = " << e.getErrorCode() << ::std::endl;
    ::std::cout << e.getMessage() << ::std::endl;
  } catch (...) {
    optimization_failed_ = true;
    ::std::cout << "Exception during optimization" << ::std::endl;
  }

  // if optimization failed, use the last generated trajectory to update the
  // current trajectory as well as the current control; the polyhedra that are
  // used are kept the same as the previous iteration
  if (optimization_failed_) {
    int64_t stamp_ms = now().nanoseconds() / 1e6;
    ::std::cout << "id: " << id_ << " stamp optimization fail : " << stamp_ms
                << ::std::endl;
    if (!traj_curr.empty()) {
      // Remove the first element
      traj_curr.erase(traj_curr.begin());
      control_curr.erase(control_curr.begin());

      // Duplicate the last element and append it
      ::std::vector<double> last_state = traj_curr.back();
      traj_curr.push_back(last_state);
      ::std::vector<double> last_control = control_curr.back();
      control_curr.push_back(last_control);

      // save the vectors
      traj_curr_ = traj_curr;
      control_curr_ = control_curr;
    }
  }

  // save computation time
  comp_time_opt_.push_back((double)(clock() - t_start) / CLOCKS_PER_SEC * 1e3);
}

void Agent::ComputeYawAngle() {
  // first get the direction as the difference between the current position and
  // the last reference point
  ::Eigen::Vector3d yaw_vec(traj_ref_curr_[yaw_idx_][0] - state_curr_[0],
                            traj_ref_curr_[yaw_idx_][1] - state_curr_[1],
                            traj_ref_curr_[yaw_idx_][2] - state_curr_[2]);

  // continue only if the vector is not close to the 0 vector
  if (yaw_vec.squaredNorm() > 0.1) {
    // project on the x-y plane
    ::Eigen::Vector3d z_vec(0, 0, 1);
    yaw_vec = yaw_vec - yaw_vec.dot(z_vec) * z_vec;

    // get ref angle
    double yaw_ref = ::std::atan2(yaw_vec(1), yaw_vec(0));

    // update current angle using a P controller and the yaw velocity
    double error_ang = yaw_ref - yaw_;
    if (error_ang > M_PI) {
      error_ang = error_ang - 2 * M_PI;
    } else if (error_ang < -M_PI) {
      error_ang = error_ang + 2 * M_PI;
    }
    yaw_ = yaw_ + k_p_yaw_ * error_ang * dt_;
  }
}

::Eigen::Quaterniond Agent::ComputeAttitude() {
  // get the thrust vector from the current acceleration + gravity (we can
  // include drag forces)
  ::Eigen::Vector3d acc(state_curr_[6], state_curr_[7], state_curr_[8]);
  ::Eigen::Vector3d g(0, 0, -9.81);
  ::Eigen::Vector3d thrust = mass_ * acc - mass_ * g;
  ::Eigen::Vector3d z_b = thrust.normalized();

  // compute x_b_inter, y_b and x_b from yaw
  ::Eigen::Vector3d x_b_inter(cos(yaw_), sin(yaw_), 0);
  ::Eigen::Vector3d y_b = z_b.cross(x_b_inter);
  ::Eigen::Vector3d x_b = y_b.cross(z_b);

  // compute the rotation matrix and return the quaternion
  ::Eigen::Matrix3d rotation_matrix;
  rotation_matrix << x_b, y_b, z_b;
  return ::Eigen::Quaterniond(rotation_matrix);
}

::std::vector<GRBLinExpr>
Agent::GetGurobiPolyhedronConstraints(LinearConstraint3D &poly, GRBLinExpr *x) {
  ::std::vector<GRBLinExpr> polyhedron_gurobi_const;
  auto A = poly.A_;
  auto b = poly.b_;
  for (int i = 0; i < A.rows(); i++) {
    GRBLinExpr lin_exp = -b[i];
    for (int j = 0; j < A.cols(); j++) {
      lin_exp = lin_exp + A(i, j) * x[j];
    }
    polyhedron_gurobi_const.push_back(lin_exp);
  }
  return polyhedron_gurobi_const;
}

void Agent::GenerateTimeAwareSafeCorridor() {
  // start clock
  clock_t t_start = clock();

  // first clear and resize the previous time aware safe corridor
  poly_const_final_vec_.clear();
  poly_const_final_vec_.resize(n_hor_);

  // go through every discrete point and add hyperplanes to the safe
  // corridor
  for (int i = 0; i < n_hor_; i++) {
    // get raw polyhedra that avoid static obstacles
    ::std::vector<LinearConstraint3D> poly_const_vec = poly_const_vec_;

    // get respective position of the current agent (we have n_hor_+1
    // states)
    Vec3f pos_curr;
    if (traj_curr_.size() > 0) {
      state_mtx_.lock();
      pos_curr = Vec3f(traj_curr_[i + 1][0], traj_curr_[i + 1][1],
                       traj_curr_[i + 1][2]);
      state_mtx_.unlock();
    } else {
      pos_curr = Vec3f(state_ini_[0], state_ini_[1], state_ini_[2]);
    }

    // go through each robot and add the hyperplane
    for (int j = 0; j < n_rob_; j++) {
      // get the last trajectory of the agent
      traj_other_mtx_[j].lock();
      auto stamp_other = traj_other_agents_[j].stamp;
      ::multi_agent_planner_msgs::msg::Trajectory traj = traj_other_agents_[j];
      traj_other_mtx_[j].unlock();

      int64_t stamp_now = now().nanoseconds() / 1e6;

      // get the stamp in milliseconds from the trajectory
      int64_t stamp_ms = stamp_other.sec * 1e3 + stamp_other.nanosec / 1e6;

      // compute the iteration number/difference; it should be 1
      long long it_other = ::std::floor(stamp_ms / (dt_ * step_plan_ * 1e3));
      long long it_now = ::std::floor(stamp_now / (dt_ * step_plan_ * 1e3));
      long long it_diff = it_now - it_other;

      int64_t stamp_diff = stamp_now - stamp_ms;

      // if j == id_, the trajectory size will be 0 so no need to check
      // that condition as well
      if (traj.states.size() != 0) {
        /* if (it_diff != 1) && i == 0) { */
        /*   ::std::cout << "stamp problem: " << stamp_diff << ::std::endl; */
        /*   ::std::cout << "stamp_now: " << stamp_now << ::std::endl; */
        /*   ::std::cout << "stamp_traj: " << stamp_ms << ::std::endl; */
        /*   ::std::cout << "stamp_traj_sec: " << stamp_other.sec <<
         * ::std::endl; */
        /*   ::std::cout << "stamp_traj_nanosec: " << stamp_other.nanosec */
        /*               << ::std::endl; */
        /*   ::std::cout << "agent id: " << j << ::std::endl; */
        /*   ::std::cout << "it_diff: " << it_diff << ::std::endl; */
        /* } */
        // get the position of the other robot
        Vec3f pos_other(traj.states[i + 1].position[0],
                        traj.states[i + 1].position[1],
                        traj.states[i + 1].position[2]);

        // get the normal vector
        Vec3f plane_normal = pos_other - pos_curr;
        Vec3f plane_normal_normalized = plane_normal.normalized();

        // get the midpoint between the positions
        Vec3f pos_mid = (pos_curr + pos_other) / 2;

        // compute the distance using the the ellipsoid function
        double angle_x_axis =
            M_PI_2 - ::std::fabs(::std::acos(plane_normal_normalized(2)));
        double t_val = ::std::atan(drone_radius_ / drone_z_offset_ *
                                   ::std::tan(angle_x_axis));
        double x_val = drone_radius_ * ::std::cos(t_val);
        double y_val = drone_z_offset_ * ::std::sin(t_val);
        double safety_dist = ::std::hypot(x_val, y_val);

        // compute the plane point
        Vec3f plane_point =
            pos_mid - ::std::min(2 * safety_dist, plane_normal.norm()) / 2 *
                          plane_normal_normalized;

        /* get the final normal vector after adding the perturbations */
        // first set up the perturbation vectors
        Vec3f up(0, 0, 1);
        Vec3f up_2(0, 1, 0);
        Vec3f right = plane_normal_normalized.cross(up) +
                      plane_normal_normalized.cross(up_2);
        Vec3f up_final = plane_normal_normalized.cross(up_2);
        // setup the perturbation variables
        double var_tmp = 0.1;
        // the perturbation that is time varying should be the same for
        // all agents to guanrantee symmetry/safety; this is done by using
        // the fact that the clocks are synchranized
        auto t_wall = ::std::chrono::high_resolution_clock::now();
        // get the time in milliseconds
        long long t_wall_ms =
            ::std::chrono::duration_cast<std::chrono::milliseconds>(
                t_wall.time_since_epoch())
                .count();
        // compute perturbation based on time; it is based on the quotient
        // of the division of the time in milliseconds by the planning step
        long long pert_int = (t_wall_ms) / (dt_ * step_plan_ * 1e3);
        // the perturbation variable need to account for the fact that every
        // new iteration, it is changing the previous hyperplanes of all the
        // trajectory which may render the trajectory infeasible; need
        // modification to account for that
        double pert = 0; // pert_int % 50 / 1000;
        // compute the final plane normal by adding the perturbations to
        // the original plane normal
        Vec3f plane_normal_final = (var_tmp + pert) * right +
                                   (var_tmp * up_final) +
                                   plane_normal_normalized;
        // generate the hyperplane and add it to the polyhedra
        Hyperplane3D hp_rob = Hyperplane3D(plane_point, plane_normal_final);
        poly_const_vec = AddHyperplane(poly_const_vec, hp_rob);
      }
    }
    // add the time aware safe corridor corresponding to the discrete
    // point i to the vectory of safe corridors
    poly_const_final_vec_[i] = poly_const_vec;
  }

  // save computation time
  comp_time_tasc_.push_back((double)(clock() - t_start) / CLOCKS_PER_SEC * 1e3);
}

::std::vector<LinearConstraint3D>
Agent::AddHyperplane(::std::vector<LinearConstraint3D> &poly_const_vec,
                     Hyperplane3D &hp) {
  ::std::vector<LinearConstraint3D> poly_const_tmp;
  for (auto &poly_i : poly_const_vec) {
    auto A_ = poly_i.A_;
    auto b_ = poly_i.b_;
    A_.conservativeResize(A_.rows() + 1, A_.cols());
    A_.row(A_.rows() - 1) = hp.n_;
    b_.conservativeResize(b_.size() + 1);
    b_(b_.size() - 1) =
        (hp.n_[0] * hp.p_[0] + hp.n_[1] * hp.p_[1] + hp.n_[2] * hp.p_[2]);
    poly_i.A_ = A_;
    poly_i.b_ = b_;
    poly_const_tmp.push_back(poly_i);
  }
  return poly_const_tmp;
}

void Agent::GenerateSafeCorridor() {
  // start clock
  clock_t t_start = clock();

  // define new polyhedra seeds for visualization
  ::std::vector<::std::vector<double>> poly_seeds_new;

  // define new contraints
  ::std::vector<LinearConstraint3D> poly_const_vec_new;

  // define vector for visualization of the polyhedra
  vec_E<Polyhedron3D> poly_vec_new;

  // if the trajectory fits inside the last polyhedron, keep that
  // polyhedron; this is to avoid some edge cases where the optimizer uses all
  // polyhedra when its not necessary and no additional polyhedra are generated
  // because the polyhedra horizon has been reached
  if (poly_const_vec_.size() > 0) {
    bool poly_used = true;
    for (int j = 0; j < traj_curr_.size(); j++) {
      Vec3f pt(traj_curr_[j][0], traj_curr_[j][1], traj_curr_[j][2]);
      if (!poly_const_vec_.back().inside(pt)) {
        poly_used = false;
        break;
      }
    }
    if (poly_used == true) {
      poly_vec_new.push_back(poly_vec_.back());
      poly_const_vec_new.push_back(poly_const_vec_.back());
      poly_seeds_new.push_back(poly_seeds_.back());
    }
  }

  // keep the polyhedra that were used in the previous optimization for the
  // trajectory generation for feasibility guarantees if this was not the first
  // optimization or the trajectory is not fully contained in the last
  // polyhedron
  if (poly_const_vec_.size() > 0 && poly_vec_new.size() == 0) {
    // insert poly and seed
    for (int i = 0; i < int(poly_used_idx_.size()); i++) {
      if (poly_used_idx_[i]) {
        poly_vec_new.push_back(poly_vec_[i]);
        poly_const_vec_new.push_back(poly_const_vec_[i]);
        poly_seeds_new.push_back(poly_seeds_[i]);
      }
    }
  }

  // generate new polyhedra using the global path until poly_hor_
  // first copy the global path and add the current state to it
  path_mtx_.lock();
  ::std::deque<::std::vector<double>> path_curr(path_curr_.begin(),
                                                path_curr_.end());
  path_mtx_.unlock();
  path_curr.push_front({state_curr_[0], state_curr_[1], state_curr_[2]});

  // then copy the voxel grid used for the generation
  voxel_grid_mtx_.lock();
  ::voxel_grid_util::VoxelGrid vg_util = voxel_grid_;
  voxel_grid_mtx_.unlock();

  // then find segment outside the polyhedra in our poly_const_vec_new by
  // walking on the path starting from the current position
  int n_poly = poly_const_vec_new.size();
  // generate new poly
  // start by sampling the path until we reach a point that is outside the
  // polyhedra that already exist and that is different that the seeds of
  // these polyhedra
  int path_idx = 1;
  ::Eigen::Vector3d curr_pt(path_curr[0][0], path_curr[0][1], path_curr[0][2]);
  // set the unknown voxels to occupied before generating the Safe Corridor
  vg_util.OccupyUnknown();
  double voxel_size = vg_util.GetVoxSize();
  ::Eigen::Vector3d origin = vg_util.GetOrigin();
  ::Eigen::Vector3i dim = vg_util.GetDim();
  while (n_poly < poly_hor_) {
    ::Eigen::Vector3d next_pt(path_curr[path_idx][0], path_curr[path_idx][1],
                              path_curr[path_idx][2]);
    ::Eigen::Vector3d diff = (next_pt - curr_pt);
    double dist_next = diff.norm();
    double samp_dist = voxel_size / 10;
    if (dist_next > samp_dist) {
      // sample the next point along the line between the current point
      // and the next point
      curr_pt = curr_pt + samp_dist * diff / dist_next;
    } else {
      // set current point to the next path point
      curr_pt = next_pt;

      // increment path index and check if the next point is the last
      // point
      path_idx = path_idx + 1;
      if (path_idx == int(path_curr.size())) {
        // we reached the final point
        break;
      }
    }

    // check if point is in at least one poly
    bool inside_at_least_one_poly = false;
    for (int i = 0; i < int(poly_const_vec_new.size()); i++) {
      if (poly_const_vec_new[i].inside(
              Vec3f(curr_pt(0), curr_pt(1), curr_pt(2)))) {
        inside_at_least_one_poly = true;
        break;
      }
    }

    // if the point is in at least one poly, continue to the next sampled
    // point on the path
    if (inside_at_least_one_poly) {
      continue;
    }

    // set curr_pt to the previous point
    ::Eigen::Vector3d seed_pt = curr_pt;
    if (dist_next > 0) {
      seed_pt = curr_pt - ::std::min(samp_dist, dist_next) * diff / dist_next;
    }

    // check if current point is not a previous seed
    Vec3i seed(int((seed_pt[0] - origin[0]) / voxel_size),
               int((seed_pt[1] - origin[1]) / voxel_size),
               int((seed_pt[2] - origin[2]) / voxel_size));

    bool is_previous_seed = false;
    for (int i = 0; i < int(poly_seeds_new.size()); i++) {
      Vec3f seed_world;
      seed_world[0] = seed[0] * voxel_size + voxel_size / 2 + origin[0];
      seed_world[1] = seed[1] * voxel_size + voxel_size / 2 + origin[1];
      seed_world[2] = seed[2] * voxel_size + voxel_size / 2 + origin[2];
      if (seed_world(0) == poly_seeds_new[i][0] &&
          seed_world(1) == poly_seeds_new[i][1] &&
          seed_world(2) == poly_seeds_new[i][2]) {
        is_previous_seed = true;
        break;
      }
    }

    // if the seed was already used for one of the polyhedra, continue to
    // the next sampled point on the path
    if (is_previous_seed) {
      continue;
    }

    /* generate polyhedron */
    bool use_cvx = use_cvx_;
    bool use_cvx_new = use_cvx_new_;
    // first check if the seed is constrained from a given direction
    if ((vg_util.IsOccupied(::Eigen::Vector3i(seed[0] - 1, seed[1], seed[2])) &&
         vg_util.IsOccupied(
             ::Eigen::Vector3i(seed[0] + 1, seed[1], seed[2]))) ||
        (vg_util.IsOccupied(::Eigen::Vector3i(seed[0], seed[1] - 1, seed[2])) &&
         vg_util.IsOccupied(
             ::Eigen::Vector3i(seed[0], seed[1] + 1, seed[2]))) ||
        (vg_util.IsOccupied(::Eigen::Vector3i(seed[0], seed[1], seed[2] - 1)) &&
         vg_util.IsOccupied(
             ::Eigen::Vector3i(seed[0], seed[1], seed[2] + 1)))) {
      use_cvx_new = true;
    }
    // first push seed into the new poly_seeds list
    Vec3f seed_world;
    seed_world[0] = seed[0] * voxel_size + voxel_size / 2 + origin[0];
    seed_world[1] = seed[1] * voxel_size + voxel_size / 2 + origin[1];
    seed_world[2] = seed[2] * voxel_size + voxel_size / 2 + origin[2];
    poly_seeds_new.push_back({seed_world(0), seed_world(1), seed_world(2)});
    // use point as seed for polyhedra
    // check if use voxel grid based method or liu's method
    Polyhedron3D poly_new;
    ::std::vector<int8_t> grid_data = vg_util.GetData();
    if (use_cvx) {
      if (use_cvx_new) {
        // if use new method
        poly_new = ::convex_decomp_lib::GetPolyOcta3DNew(
            seed, grid_data, dim, n_it_decomp_, voxel_size, -(n_poly + 1),
            origin);
      } else {
        // use original method
        poly_new = ::convex_decomp_lib::GetPolyOcta3D(seed, grid_data, dim,
                                                      n_it_decomp_, voxel_size,
                                                      -(n_poly + 1), origin);
      }
    } else {
      // TODO (but not essential) : use liu's method
    }

    // increment the number of polyhedra
    n_poly = n_poly + 1;

    // convert polyhedron to linear constraints and add it to the poly
    // constraints vector; first gets Vec3f point and second gets Vec3f
    // normals defining hyperplanes
    auto poly_inf = poly_new.cal_normals();
    int size_poly = poly_inf.size();
    MatDNf<3> A_poly(size_poly, 3);
    VecDf b_poly(size_poly);
    for (int i = 0; i < size_poly; i++) {
      A_poly.row(i) = poly_inf[i].second;
      b_poly(i) = poly_inf[i].first.dot(poly_inf[i].second);
    }
    poly_vec_new.push_back(poly_new);
    poly_const_vec_new.push_back(LinearConstraint3D(A_poly, b_poly));
  }

  // save polyhedra and seeds
  poly_vec_ = poly_vec_new;
  poly_const_vec_ = poly_const_vec_new;
  poly_seeds_ = poly_seeds_new;

  // save computation time
  comp_time_sc_.push_back((double)(clock() - t_start) / CLOCKS_PER_SEC * 1e3);
}

void Agent::GenerateReferenceTrajectory() {
  // get reference path
  path_mtx_.lock();
  ::std::vector<::std::vector<double>> path_curr = path_curr_;
  path_mtx_.unlock();

  // find the starting point from the previous reference trajectory
  int traj_ref_start_idx = 0;
  ::std::vector<double> starting_point;
  ::std::vector<double> last_point;
  if (traj_ref_curr_.size() > 0 && !reset_path_) {
    traj_ref_mtx_.lock();
    if (increment_traj_ref_) {
      starting_point = {traj_ref_curr_[1][0], traj_ref_curr_[1][1],
                        traj_ref_curr_[1][2]};
      /* starting_point = starting_point_; */
    } else {
      starting_point = {traj_ref_curr_[0][0], traj_ref_curr_[0][1],
                        traj_ref_curr_[0][2]};
    }
    last_point = traj_ref_curr_.back();
    traj_ref_mtx_.unlock();
  } else {
    starting_point = path_curr[0];
    last_point = path_curr[0];
    traj_ref_start_idx = -1;
  }

  int start_idx = 0;
  /***** Option 1 - best option so far *****/
  // find the segment of the path that contains the starting point
  for (int i = 0; i < int(path_curr.size() - 1); i++) {
    ::std::vector<double> s_1 = path_curr[i];
    ::std::vector<double> s_2 = path_curr[i + 1];
    if (IsOnSegment(starting_point, s_1, s_2)) {
      start_idx = i + 1;
      break;
    }
  }
  // remove first start_idx elements from the path_curr (better to use
  // deque structure but here the vector is small), and add the starting point
  // at the beginning
  ::std::vector<::std::vector<double>> path_samp;
  path_samp.push_back(starting_point);
  for (int i = start_idx; i < int(path_curr.size()); i++) {
    path_samp.push_back(path_curr[i]);
  }

  /***** Option 2 *****/
  // find the segment of the path that contains the last point
  /* for (int i = 0; i < int(path_curr.size() - 1); i++) { */
  /*   ::std::vector<double> s_1 = path_curr[i]; */
  /*   ::std::vector<double> s_2 = path_curr[i + 1]; */
  /*   if (IsOnSegment(last_point, s_1, s_2)) { */
  /*     start_idx = i + 1; */
  /*     break; */
  /*   } */
  /* } */
  // build path from reference trajectory and start_idx
  /* ::std::vector<::std::vector<double>> path_samp; */
  /* if (traj_ref_start_idx >= 0 && traj_ref_curr_.size() > 1) { */
  /*   path_samp.insert(path_samp.begin(), */
  /*                    traj_ref_curr_.begin() + traj_ref_start_idx, */
  /*                    traj_ref_curr_.end() - 1); */
  /* } */
  /* path_samp.push_back(last_point); */
  /* for (int i = start_idx; i < int(path_curr.size()); i++) { */
  /*   path_samp.push_back(path_curr[i]); */
  /* } */

  // sample from the path the reference trajectory
  ::std::vector<::std::vector<double>> traj_ref_curr = SamplePath(path_samp);

  // keep only the points that are in the free voxels
  // TODO: test performance with and without
  traj_ref_curr = KeepOnlyFreeReference(traj_ref_curr);

  // generate velocity reference from the reference path
  if (traj_ref_curr.size() > 1) {
    double v_x, v_y, v_z;
    for (int i = 0; i < int(traj_ref_curr.size()) - 1; i++) {
      double dist = GetDistanceSquared(traj_ref_curr[i], traj_ref_curr[i + 1]);
      dist = ::std::sqrt(dist);
      if (dist > 1e-2) {
        v_x =
            path_vel_ * (traj_ref_curr[i][0] - traj_ref_curr[i + 1][0]) / dist;
        v_y =
            path_vel_ * (traj_ref_curr[i][1] - traj_ref_curr[i + 1][1]) / dist;
        v_z =
            path_vel_ * (traj_ref_curr[i][2] - traj_ref_curr[i + 1][2]) / dist;
      } else {
        v_x = 0;
        v_y = 0;
        v_z = 0;
      }
      traj_ref_curr[i].insert(traj_ref_curr[i].end(), {v_x, v_y, v_z});
    }
    traj_ref_curr.back().insert(traj_ref_curr.back().end(), {v_x, v_y, v_z});
  }

  // save the reference trajectory
  traj_ref_mtx_.lock();
  traj_ref_curr_ = traj_ref_curr;
  traj_ref_mtx_.unlock();
}

::std::vector<::std::vector<double>>
Agent::RemoveZigZagSegments(::std::vector<::std::vector<double>> path) {
  // get voxel grid to check if line is clear when removing zigzags
  voxel_grid_mtx_.lock();
  ::voxel_grid_util::VoxelGrid vg_util = voxel_grid_;
  voxel_grid_mtx_.unlock();

  // remove unnecessary points that are in the middle of 2 segments forming an
  // acute angle
  for (int i = 0; i < int(path.size()) - 2; i++) {
    ::std::vector<double> sg_1 = {path[i][0] - path[i + 1][0],
                                  path[i][1] - path[i + 1][1],
                                  path[i][2] - path[i + 1][2]};
    ::std::vector<double> sg_2 = {path[i + 1][0] - path[i + 2][0],
                                  path[i + 1][1] - path[i + 2][1],
                                  path[i + 1][2] - path[i + 2][2]};
    if (DotProduct(sg_1, sg_2) < 0 ||
        (remove_corners_ && DotProduct(sg_1, sg_2) == 0)) {
      // if acute angle, check if the line is clear between the points that
      // will remain after removing the midpoint of the 2 segments; first
      // transform point to voxel coordinates
      ::Eigen::Vector3d pt_1(path[i][0], path[i][1], path[i][2]);
      ::Eigen::Vector3d pt_2(path[i + 2][0], path[i + 2][1], path[i + 2][2]);
      ::Eigen::Vector3d pt_1_local = vg_util.GetCoordLocal(pt_1);
      ::Eigen::Vector3d pt_2_local = vg_util.GetCoordLocal(pt_2);
      if (::path_finding_util::IsLineClear(pt_1_local, pt_2_local, vg_util,
                                           (pt_1_local - pt_2_local).norm(),
                                           false)) {
        path.erase(path.begin() + i + 1);
        i = ::std::max(0, i - 2);
      }
    }
  }
  return path;
}

::std::vector<std::vector<double>>
Agent::SamplePath(::std::vector<::std::vector<double>> &path) {
  // create empty reference traj and add the first point to it
  ::std::vector<::std::vector<double>> traj_ref;

  // check if path has less than 2 points
  if (path.size() < 2) {
    // check if it has one element
    if (path.size() == 1) {
      // return the point as a reference and set velocity reference to 0
      for (int i = 0; i < n_hor_; i++) {
        traj_ref.push_back(path[0]);
      }
    } else {
      RCLCPP_ERROR(get_logger(),
                   "the reference path used for sampling is empty");
    }
    return traj_ref;
  }

  // compute path sampling velocity using the path and the potential field
  path_vel_ = ComputePathVelocity(path);
  /* RCLCPP_INFO(get_logger(), "path_vel: %f", path_vel_); */

  // compute sampling distance from path velocity and time step
  double samp_dist = path_vel_ * dt_;

  // path_idx indicates the index of the path point we are considering
  int path_idx = 1;

  // ref_idx indicates the number of ref points sampled so far
  int ref_idx = 0;

  ::Eigen::Vector3d curr_pt(path[0][0], path[0][1], path[0][2]);
  traj_ref.push_back(path.front());
  double limit_dist = samp_dist;
  while (ref_idx < n_hor_) {
    // compute distance to the next point
    ::Eigen::Vector3d next_pt(path[path_idx][0], path[path_idx][1],
                              path[path_idx][2]);
    ::Eigen::Vector3d diff = (next_pt - curr_pt);
    double dist_next = diff.norm();
    if (dist_next > limit_dist) {
      // sample the next point along the line between the current point and
      // the next point
      curr_pt = curr_pt + limit_dist * diff / dist_next;
      traj_ref.push_back({curr_pt(0), curr_pt(1), curr_pt(2)});
      ref_idx = ref_idx + 1;

      // reset limit distance but with deceleration
      limit_dist = ::std::max(0.0, samp_dist - path_vel_dec_ * dt_);
    } else {
      // set current point to the next path point
      curr_pt = next_pt;

      // increment path index and check if the next point is the last point
      path_idx = path_idx + 1;
      if (path_idx == int(path.size())) {
        // fill the remaining points of traj_ref with this point and return it
        for (int i = ref_idx; i < n_hor_; i++) {
          traj_ref.push_back(path.back());
        }
        return traj_ref;
      }

      // remove the traversed distance from the limit distance
      limit_dist = limit_dist - dist_next;
    }
  }

  // return reference trajectory
  return traj_ref;
}

::std::vector<::std::vector<double>>
Agent::KeepOnlyFreeReference(::std::vector<::std::vector<double>> &traj_ref) {
  // get the latest version of the voxel grid
  voxel_grid_mtx_.lock();
  ::voxel_grid_util::VoxelGrid vg_util = voxel_grid_;
  voxel_grid_mtx_.unlock();

  // declare the final reference trajectory
  ::std::vector<::std::vector<double>> traj_ref_final;
  traj_ref_final.push_back(traj_ref.front());

  // go through each point of the trajectory and check if it is free; once we
  // encounter an unknown voxel then take the last free point and duplicate it
  // until the end
  for (int i = 1; i < traj_ref.size(); i++) {
    ::std::vector<double> pt = traj_ref[i];
    int8_t vox_value =
        vg_util.GetVoxelGlobal(::Eigen::Vector3d(pt[0], pt[1], pt[2]));
    if (vox_value == ENV_BUILDER_UNK || vox_value == ENV_BUILDER_OCC) {
      // we fill the rest of traj_ref_final with the last point
      for (int j = i; j < traj_ref.size(); j++) {
        traj_ref_final.push_back(traj_ref_final.back());
      }
      return traj_ref_final;
    }
    traj_ref_final.push_back(pt);
  }
  return traj_ref_final;
}

double Agent::ComputePathVelocity(::std::vector<::std::vector<double>> &path) {
  // define path velocity
  double path_vel = path_vel_max_;

  // path start eigen vector
  ::Eigen::Vector3d path_start(path[0][0], path[0][1], path[0][2]);

  // get the latest version of the voxel grid
  voxel_grid_mtx_.lock();
  ::voxel_grid_util::VoxelGrid vg_util = voxel_grid_;
  voxel_grid_mtx_.unlock();

  // go through each segment of the path and compute its limit on the path
  for (int i = 0; i < int(path.size()) - 1; i++) {
    ::Eigen::Vector3d start = vg_util.GetCoordLocal(
        ::Eigen::Vector3d(path[i][0], path[i][1], path[i][2]));
    ::Eigen::Vector3d end = vg_util.GetCoordLocal(
        ::Eigen::Vector3d(path[i + 1][0], path[i + 1][1], path[i + 1][2]));
    ::Eigen::Vector3d collision_pt;
    ::std::vector<::Eigen::Vector3d> visited_points;
    double max_dist_raycast = (start - end).norm();
    bool line_clear = ::path_finding_util::IsLineClear(
        start, end, vg_util, max_dist_raycast, collision_pt, visited_points);

    // add the start to the visited points
    visited_points.push_back(start);
    if (line_clear) {
      /* go through each visited voxel (integer coordinates in the local grid)
       and compute the path velocity limit of each one; if it's smaller than
       path_vel, then set path_vel to the new minimum path velocity; this does
       not include the start point  of the raycasting */
      for (const ::Eigen::Vector3d &pt : visited_points) {
        // get the value of the voxel/visited point and the distance of the
        // voxel to the start point
        /* ::std::cout << "visited pt: " << pt.transpose() << ::std::endl; */
        double voxel_val = double(vg_util.GetVoxelInt(pt));
        if (voxel_val == -1) {
          voxel_val = 100;
        }

        double dist_start = (path_start - pt).norm() * vg_util.GetVoxSize();

        // use the heuristic to compute the path velocity limit
        double path_vel_tmp = GetVelocityLimit(voxel_val, dist_start);

        /* ::std::cout << "voxel_val: " << voxel_val << ::std::endl; */
        /* ::std::cout << "dist_start: " << dist_start << ::std::endl; */
        /* ::std::cout << "path_vel_tmp: " << path_vel_tmp << ::std::endl; */

        // update the path_vel
        if (path_vel_tmp < path_vel) {
          path_vel = path_vel_tmp;
        }
      }
    } else {
      // look at the collision point and use it to limit the speed
      int8_t voxel_val = vg_util.GetVoxelInt(collision_pt);
      double dist_start = (start - collision_pt).norm();
      double path_vel_tmp = GetVelocityLimit(voxel_val, dist_start);
      if (path_vel_tmp < path_vel) {
        path_vel = path_vel_tmp;
      }

      // stop the loop after the collision
      break;
    }
  }

  return path_vel;
}

double Agent::GetVelocityLimit(double occ_val, double dist_start) {
  // compute linear factor
  double alpha = 1 - (1 - 1 / exp(sens_pot_ * occ_val)) *
                         (1 / exp(sens_dist_ * dist_start));
  double path_vel = path_vel_min_ + (path_vel_max_ - path_vel_min_) * alpha;
  return path_vel;
}

void Agent::ClearBoundary(::voxel_grid_util::VoxelGrid &voxel_grid) {
  ::Eigen::Vector3i dim = voxel_grid.GetDim();
  int x_max = dim[0] - 1;
  int y_max = dim[1] - 1;
  int z_max = dim[2] - 1;

  for (int j = 0; j <= y_max; j++) {
    for (int k = 0; k <= z_max; k++) {
      ::Eigen::Vector3i pt_1(0, j, k);
      ::Eigen::Vector3i pt_2(x_max, j, k);
      voxel_grid.SetVoxelInt(pt_1, 0);
      voxel_grid.SetVoxelInt(pt_2, 0);
    }
  }

  for (int i = 0; i <= x_max; i++) {
    for (int k = 0; k <= z_max; k++) {
      ::Eigen::Vector3i pt_1(i, 0, k);
      ::Eigen::Vector3i pt_2(i, y_max, k);
      voxel_grid.SetVoxelInt(pt_1, 0);
      voxel_grid.SetVoxelInt(pt_2, 0);
    }
  }

  // for now dont clear the roof and the floor because it causes some
  // issues due to the fact that the z dimension of the grid is much
  // smaller then the other dimensions
  /* for (int i = 0; i < x_max; i++) { */
  /*   for (int j = 0; j < y_max; j++) { */
  /*     ::Eigen::Vector3i pt_1(i, j, 0); */
  /*     ::Eigen::Vector3i pt_2(i, j, z_max); */
  /*     voxel_grid.SetVoxelInt(pt_1, 0); */
  /*     voxel_grid.SetVoxelInt(pt_2, 0); */
  /*   } */
  /* } */
}

double Agent::GetDistanceSquared(::std::vector<double> &p1,
                                 ::std::vector<double> &p2) {
  double distance = (p1[0] - p2[0]) * (p1[0] - p2[0]) +
                    (p1[1] - p2[1]) * (p1[1] - p2[1]) +
                    (p1[2] - p2[2]) * (p1[2] - p2[2]);
  return distance;
}

bool Agent::IsOnSegment(::std::vector<double> &pt, ::std::vector<double> &s_1,
                        ::std::vector<double> &s_2) {
  // compute distances between all points
  double dist_pt_s_1 = ::std::sqrt(GetDistanceSquared(pt, s_1));
  double dist_pt_s_2 = ::std::sqrt(GetDistanceSquared(pt, s_2));
  double dist_s_1_s_2 = ::std::sqrt(GetDistanceSquared(s_1, s_2));

  // check if the distance are such the pt is on the segment between s_1 and
  // s_2
  if (::std::abs(dist_pt_s_1 + dist_pt_s_2 - dist_s_1_s_2) < 1e-6) {
    // check the dot product between s_1-pt and s_2-pt is negative
    ::std::vector<double> s_1_pt = {pt[0] - s_1[0], pt[1] - s_1[1],
                                    pt[2] - s_1[2]};
    ::std::vector<double> s_2_pt = {pt[0] - s_2[0], pt[1] - s_2[1],
                                    pt[2] - s_2[2]};
    if (DotProduct(s_1_pt, s_2_pt) <= 0) {
      return true;
    }
  }
  return false;
}

double Agent::DotProduct(::std::vector<double> &v_1,
                         ::std::vector<double> &v_2) {
  return v_1[0] * v_2[0] + v_1[1] * v_2[1] + v_1[2] * v_2[2];
}

::std::vector<double>
Agent::GetIntermediateGoal(::std::vector<double> &goal,
                           ::voxel_grid_util::VoxelGrid &voxel_grid) {
  // get goal position in grid frame
  ::Eigen::Vector3d goal_grid_frame;
  ::Eigen::Vector3d origin = voxel_grid.GetOrigin();
  goal_grid_frame[0] = goal[0] - origin[0];
  goal_grid_frame[1] = goal[1] - origin[1];
  goal_grid_frame[2] = goal[2] - origin[2];

  // get real dimension of the grid
  ::Eigen::Vector3i dim = voxel_grid.GetDim();
  double voxel_size = voxel_grid.GetVoxSize();
  ::Eigen::Vector3d dim_real(dim[0] * voxel_size, dim[1] * voxel_size,
                             dim[2] * voxel_size);

  // first test if goal is inside the grid
  if (goal_grid_frame(0) < dim_real(0) && goal_grid_frame(0) > 0 &&
      goal_grid_frame(1) < dim_real(1) && goal_grid_frame(1) > 0 &&
      goal_grid_frame(2) < dim_real(2) && goal_grid_frame(2) > 0) {
    return goal;
  } else {
    // if the goal is outside the grid find the intersection with the grid
    // first find the center of the grid
    ::Eigen::Vector3d center_pos((dim[0] / 2 + 0.5) * voxel_size,
                                 (dim[1] / 2 + 0.5) * voxel_size,
                                 (dim[2] / 2 + 0.5) * voxel_size);

    // then find the direction between the center of the grid and the goal
    ::Eigen::Vector3d ray_dir = goal_grid_frame - center_pos;
    ray_dir.normalize();

    // get intersection with the edge of the grid
    double min_dim = voxel_size * ::std::min(std::min(dim[0], dim[1]), dim[2]);
    ::Eigen::Vector3d sampled_point = center_pos + (min_dim / 2) * ray_dir;
    int n_it = 0;
    while (n_it < 100) {
      if (sampled_point(0) > dim_real(0) || sampled_point(1) > dim_real(1) ||
          sampled_point(2) > dim_real(2) || sampled_point(0) < 0 ||
          sampled_point(1) < 0 || sampled_point(2) < 0) {
        sampled_point = sampled_point - 0.5 * voxel_size * ray_dir;
        break;
      }
      sampled_point = sampled_point + 0.5 * voxel_size * ray_dir;
    }
    ::std::vector<double> goal_fin = {sampled_point[0] + origin[0],
                                      sampled_point[1] + origin[1],
                                      sampled_point[2] + origin[2]};
    return goal_fin;
  }
}

void Agent::SaveAndDisplayCompTime(::std::vector<double> &comp_time,
                                   ::std::string &filename) {
  if (save_stats_) {
    // save file
    ::std::ofstream myfile;
    myfile.open(filename);
    for (int i = 0; i < int(comp_time.size()); i++) {
      myfile << ::std::fixed << comp_time[i] << ",";
    }
    myfile.close();
  }

  // compute and display min, max and mean
  ::std::cout << filename << ": ";
  double max_t = 0;
  double min_t = 1e10;
  double sum_t = 0;
  for (int i = 0; i < int(comp_time.size()); i++) {
    if (comp_time[i] > max_t) {
      max_t = comp_time[i];
    }
    if (comp_time[i] < min_t) {
      min_t = comp_time[i];
    }
    sum_t = sum_t + comp_time[i];
  }
  ::std::cout << ::std::endl << "mean: " << sum_t / comp_time.size();
  ::std::cout << ::std::endl << "max: " << max_t;
  ::std::cout << ::std::endl << "min: " << min_t << ::std::endl;
}

void Agent::SaveStateHistory() {
  if (save_stats_) {
    // one each row, first save the time then the state
    ::std::string filename = "state_hist_" + ::std::to_string(id_) + ".csv";
    ::std::ofstream myfile;
    myfile.open(filename);
    for (int i = 0; i < int(state_hist_.size()); i++) {
      ::std::vector<double> state = state_hist_[i];
      myfile << ::std::fixed << state_hist_stamp_[i] << ",";
      for (int j = 0; j < int(state.size()); j++) {
        myfile << ::std::fixed << state[j];
        if (j != int(state.size()) - 1) {
          myfile << ",";
        }
      }
      myfile << ::std::endl;
    }
    myfile.close();
  }

  // display velocity stats
  double vel_average = 0;
  double vel_max = 0;
  for (auto state : state_hist_) {
    double vel = ::std::sqrt(state[3] * state[3] + state[4] * state[4] +
                             state[5] * state[5]);
    vel_average += vel;
    if (vel > vel_max) {
      vel_max = vel;
    }
  }
  vel_average /= double(state_hist_.size());
  ::std::cout << ::std::endl << "velocity for agent: " << id_;
  ::std::cout << ::std::endl << "mean: " << vel_average;
  ::std::cout << ::std::endl << "max: " << vel_max << ::std::endl;
}

void Agent::SaveAndDisplayCommunicationLatency() {
  // go through all agents and compute the average and max communication
  // latency
  ::std::vector<double> com_latency_mean;
  com_latency_mean.resize(n_rob_);
  ::std::vector<double> com_latency_max;
  com_latency_max.resize(n_rob_);

  double total_mean = 0;
  double total_max = 0;
  for (int i = 0; i < n_rob_; i++) {
    if (i != id_) {
      ::std::vector<double> com_latency_i = com_latency_ms_[i];
      double lat_mean = 0;
      double lat_max = 0;
      for (auto lat : com_latency_i) {
        lat_mean = lat_mean + lat;
        if (lat > lat_max) {
          lat_max = lat;
        }
      }
      com_latency_mean[i] = lat_mean / double(com_latency_i.size());
      com_latency_max[i] = lat_max;
      total_mean = total_mean + com_latency_mean[i];
      if (com_latency_max[i] > total_max) {
        total_max = com_latency_max[i];
      }
    }
  }
  total_mean = total_mean / (n_rob_ - 1);

  ::std::cout << "communication latency (ms) for agent " << id_
              << ": mean: " << total_mean << " max: " << total_max
              << ::std::endl;

  if (save_stats_) {
    ::std::string filename = "com_latency_" + ::std::to_string(id_) + ".csv";
    ::std::ofstream myfile;
    myfile.open(filename);
    for (int i = 0; i < n_rob_; i++) {
      if (i != id_) {
        ::std::vector<double> com_latency_i = com_latency_ms_[i];
        for (auto lat : com_latency_i) {
          myfile << ::std::fixed << lat << ",";
        }
        myfile << ::std::endl;
      }
    }
    myfile.close();
  }
}

GRBModel Agent::InitializeGurobi(GRBEnv &env) {
  if (gurobi_verbose_) {
    env.set(GRB_IntParam_OutputFlag, 0);
  }
  env.start();
  return GRBModel(env);
}

void Agent::CreateGurobiModel() {
  // create temporary variables
  ::std::vector<GRBVar> tmp_vars;

  // add state variables to model
  for (int i = 0; i < (n_hor_ + 1); i++) {
    for (int j = 0; j < n_x_; j++) {
      if (i == n_hor_ && j >= 3) { //  && j<=5
        tmp_vars.push_back(
            model_.addVar(0.0, 0.0, 0.0, GRB_CONTINUOUS,
                          "x_" + ::std::to_string(i) + ::std::to_string(j)));
      } else {
        tmp_vars.push_back(
            model_.addVar(x_lb_[j], x_ub_[j], 0.0, GRB_CONTINUOUS,
                          "x_" + ::std::to_string(i) + ::std::to_string(j)));
      }
    }
    x_grb_.push_back(tmp_vars);
    tmp_vars.clear();
  }

  // add control variable to model + add them to the objective function
  for (int i = 0; i < n_hor_; i++) {
    for (int j = 0; j < n_u_; j++) {
      tmp_vars.push_back(
          model_.addVar(u_lb_[j], u_ub_[j], 0.0, GRB_CONTINUOUS,
                        "u_" + ::std::to_string(i) + ::std::to_string(j)));
      obj_grb_ = obj_grb_ + r_u_ * tmp_vars[j] * tmp_vars[j];
    }
    u_grb_.push_back(tmp_vars);
    tmp_vars.clear();
  }

  // add binary variables to model
  for (int i = 0; i < n_hor_; i++) {
    for (int j = 0; j < poly_hor_; j++) {
      tmp_vars.push_back(
          model_.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_BINARY,
                        "b_" + ::std::to_string(i) + ::std::to_string(j)));
    }
    b_grb_.push_back(tmp_vars);
    tmp_vars.clear();
  }

  // add dyn constraints
  GRBLinExpr x_expr[n_x_];
  for (int i = 0; i < n_hor_; i++) {
    GRBLinExpr mod_final[n_x_];
    for (int j = 0; j < n_x_; j++) {
      x_expr[j] = GRBLinExpr(x_grb_[i][j]);
    }
    GRBLinExpr *k1 = ModelODE(x_expr, u_grb_[i]);
    if (rk4_ == true) {
      GRBLinExpr tmp[n_x_];
      for (int j = 0; j < n_x_; j++) {
        tmp[j] = x_expr[j] + (dt_ / 2) * k1[j];
      }
      GRBLinExpr *k2 = ModelODE(tmp, u_grb_[i]);
      for (int j = 0; j < n_x_; j++) {
        tmp[j] = x_expr[j] + (dt_ / 2) * k2[j];
      }
      GRBLinExpr *k3 = ModelODE(tmp, u_grb_[i]);
      for (int j = 0; j < n_x_; j++) {
        tmp[j] = x_expr[j] + dt_ * k3[j];
      }
      GRBLinExpr *k4 = ModelODE(tmp, u_grb_[i]);
      for (int j = 0; j < n_x_; j++) {
        mod_final[j] = (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]) / 6;
      }
    } else {
      for (int j = 0; j < n_x_; j++) {
        mod_final[j] = k1[j];
      }
    }

    for (int j = 0; j < n_x_; j++) {
      model_.addConstr(GRBLinExpr(x_grb_[i + 1][j], 1.0) ==
                           GRBLinExpr(x_grb_[i][j], 1.0) + dt_ * mod_final[j],
                       "dyn_constr_" + ::std::to_string(i) +
                           ::std::to_string(j));
    }
  }
}

GRBLinExpr *Agent::ModelODE(GRBLinExpr *x_expr, ::std::vector<GRBVar> &u_i) {
  GRBLinExpr *res = new GRBLinExpr[9];
  res[0] = x_expr[3];
  res[1] = x_expr[4];
  res[2] = x_expr[5];
  res[3] = x_expr[6] - drag_coeff_[0] * x_expr[3];
  res[4] = x_expr[7] - drag_coeff_[1] * x_expr[4];
  res[5] = x_expr[8] - drag_coeff_[2] * x_expr[5];
  res[6] = u_i[0];
  res[7] = u_i[1];
  res[8] = u_i[2];
  return res;
}

void Agent::InitializePlannerParameters() {
  // initialze lower and upper bounds
  if (n_x_ == 6) {
    // acceleration control
    x_lb_ = {-GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY,
             -max_vel_,     -max_vel_,     -max_vel_};
    x_ub_ = {GRB_INFINITY, GRB_INFINITY, GRB_INFINITY,
             max_vel_,     max_vel_,     max_vel_};
    u_lb_ = {min_acc_xy_, min_acc_xy_, min_acc_z_};
    u_ub_ = {max_acc_xy_, max_acc_xy_, max_acc_z_};
  } else if (n_x_ == 9) {
    // jerk control
    x_lb_ = {-GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -max_vel_, -max_vel_,
             -max_vel_,     min_acc_xy_,   min_acc_xy_,   min_acc_z_};
    x_ub_ = {GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, max_vel_,  max_vel_,
             max_vel_,     max_acc_xy_,  max_acc_xy_,  max_acc_z_};
    u_lb_ = {-max_jerk_, -max_jerk_, -max_jerk_};
    u_ub_ = {max_jerk_, max_jerk_, max_jerk_};
  }
}

void Agent::DeclareRosParameters() {
  // declare grid parameters
  declare_parameter("get_grid_service_name",
                    "/env_builder_node/get_voxel_grid");
  declare_parameter("voxel_grid_range", ::std::vector<double>(3, 10.0));
  declare_parameter("publish_voxel_grid", false);
  declare_parameter("voxel_grid_update_period", 0.1);
  declare_parameter("use_mapping_util", true);
  declare_parameter("gurobi_verbose", true);

  // declare planner paramters
  declare_parameter("topic_name", "agent");
  declare_parameter("world_frame", "world");
  declare_parameter("n_rob", 1);
  declare_parameter("id", 0);
  declare_parameter("n_x", 9);
  declare_parameter("n_u", 3);
  declare_parameter("n_hor", 7);
  declare_parameter("dt", 0.1);
  declare_parameter("path_vel_min", 4.5);
  declare_parameter("path_vel_max", 4.5);
  declare_parameter("sens_dist", 1.0);
  declare_parameter("sens_pot", 1.0);
  declare_parameter("path_vel_dec", 0.1);
  declare_parameter("traj_ref_points_to_keep", 10);
  declare_parameter("rk4", false);
  declare_parameter("step_plan", 1);
  declare_parameter("thresh_dist", 0.2);
  declare_parameter("poly_hor", 3);
  declare_parameter("n_it_decomp", 42);
  declare_parameter("use_cvx", true);
  declare_parameter("use_cvx_new", false);
  declare_parameter("drone_radius", 0.3);
  declare_parameter("drone_z_offset", 0.3);
  declare_parameter("path_infl_dist", 0.3);
  declare_parameter("com_latency", 0.0);
  declare_parameter("r_u", 0.01);
  declare_parameter("r_x", ::std::vector<double>(9, 0.0));
  declare_parameter("r_n", ::std::vector<double>(9, 0.0));
  declare_parameter("max_vel", 9.5);
  declare_parameter("max_acc_z", 30.0);
  declare_parameter("min_acc_z", -30.0);
  declare_parameter("max_acc_xy", 30.0);
  declare_parameter("min_acc_xy", -30.0);
  declare_parameter("max_jerk", 60.0);
  declare_parameter("mass", 1.0);
  declare_parameter("yaw_idx", 3);
  declare_parameter("k_p_yaw", 1.0);
  declare_parameter("drag_coeff", ::std::vector<double>(3, 0.0));
  declare_parameter("state_ini", ::std::vector<double>(6, 0.0));
  declare_parameter("goal", ::std::vector<double>(3, 0.0));
  declare_parameter("planner_verbose", false);
  declare_parameter("save_stats", false);
  declare_parameter("dmp_search_rad", 0.0);
  declare_parameter("dmp_n_it", 1);
  declare_parameter("path_planning_period", 0.1);
  declare_parameter("remove_corners", false);
}

void Agent::InitializeRosParameters() {
  // initialize grid service subscription
  get_grid_service_name_ = get_parameter("get_grid_service_name").as_string();
  voxel_grid_range_ = get_parameter("voxel_grid_range").as_double_array();
  publish_voxel_grid_ = get_parameter("publish_voxel_grid").as_bool();
  voxel_grid_update_period_ =
      get_parameter("voxel_grid_update_period").as_double();
  use_mapping_util_ = get_parameter("use_mapping_util").as_bool();
  gurobi_verbose_ = get_parameter("gurobi_verbose").as_bool();

  // initialize planner paramters
  topic_name_ = get_parameter("topic_name").as_string();
  world_frame_ = get_parameter("world_frame").as_string();
  n_rob_ = get_parameter("n_rob").as_int();
  id_ = get_parameter("id").as_int();
  n_x_ = get_parameter("n_x").as_int();
  n_u_ = get_parameter("n_u").as_int();
  n_hor_ = get_parameter("n_hor").as_int();
  dt_ = get_parameter("dt").as_double();
  path_vel_min_ = get_parameter("path_vel_min").as_double();
  path_vel_max_ = get_parameter("path_vel_max").as_double();
  sens_dist_ = get_parameter("sens_dist").as_double();
  sens_pot_ = get_parameter("sens_pot").as_double();
  path_vel_dec_ = get_parameter("path_vel_dec").as_double();
  traj_ref_points_to_keep_ = get_parameter("traj_ref_points_to_keep").as_int();
  rk4_ = get_parameter("rk4").as_bool();
  step_plan_ = get_parameter("step_plan").as_int();
  thresh_dist_ = get_parameter("thresh_dist").as_double();
  poly_hor_ = get_parameter("poly_hor").as_int();
  n_it_decomp_ = get_parameter("n_it_decomp").as_int();
  use_cvx_ = get_parameter("use_cvx").as_bool();
  use_cvx_new_ = get_parameter("use_cvx_new").as_bool();
  drone_radius_ = get_parameter("drone_radius").as_double();
  drone_z_offset_ = get_parameter("drone_z_offset").as_double();
  path_infl_dist_ = get_parameter("path_infl_dist").as_double();
  com_latency_ = get_parameter("com_latency").as_double();
  r_u_ = get_parameter("r_u").as_double();
  r_x_ = get_parameter("r_x").as_double_array();
  r_n_ = get_parameter("r_n").as_double_array();
  max_vel_ = get_parameter("max_vel").as_double();
  max_acc_z_ = get_parameter("max_acc_z").as_double();
  min_acc_z_ = get_parameter("min_acc_z").as_double();
  max_acc_xy_ = get_parameter("max_acc_xy").as_double();
  min_acc_xy_ = get_parameter("min_acc_xy").as_double();
  max_jerk_ = get_parameter("max_jerk").as_double();
  drag_coeff_ = get_parameter("drag_coeff").as_double_array();
  mass_ = get_parameter("mass").as_double();
  yaw_idx_ = get_parameter("yaw_idx").as_int();
  k_p_yaw_ = get_parameter("k_p_yaw").as_double();
  state_ini_ = get_parameter("state_ini").as_double_array();
  goal_curr_ = get_parameter("goal").as_double_array();
  planner_verbose_ = get_parameter("planner_verbose").as_bool();
  save_stats_ = get_parameter("save_stats").as_bool();
  dmp_search_rad_ = get_parameter("dmp_search_rad").as_double();
  dmp_n_it_ = get_parameter("dmp_n_it").as_int();
  path_planning_period_ = get_parameter("path_planning_period").as_double();
  remove_corners_ = get_parameter("remove_corners").as_bool();
}

void Agent::VoxelGridResponseCallback(
    ::rclcpp::Client<::env_builder_msgs::srv::GetVoxelGrid>::SharedFuture
        future) {
  auto status = future.wait_for(::std::chrono::seconds(1));
  if (status == ::std::future_status::ready) {
    // store the data descriptions
    ::env_builder_msgs::msg::VoxelGridStamped voxel_grid_stamped =
        future.get()->voxel_grid_stamped;
    voxel_grid_mtx_.lock();
    voxel_grid_ =
        ::mapping_util::ConvertVGMsgToVGUtil(voxel_grid_stamped.voxel_grid);
    voxel_grid_mtx_.unlock();

    voxel_grid_ready_ = true;
    if (planner_verbose_) {
      RCLCPP_INFO(get_logger(), "Received the voxel grid");
    }
    if (publish_voxel_grid_) {
      PublishVoxelGrid();
    }
  }
}

void Agent::GetVoxelGridAsync() {
  auto request =
      ::std::make_shared<::env_builder_msgs::srv::GetVoxelGrid::Request>();
  if (state_curr_.empty()) {
    request->position[0] = state_ini_[0];
    request->position[1] = state_ini_[1];
    request->position[2] = state_ini_[2];
  } else {
    request->position[0] = state_curr_[0];
    request->position[1] = state_curr_[1];
    request->position[2] = state_curr_[2];
  }
  request->range[0] = voxel_grid_range_[0];
  request->range[1] = voxel_grid_range_[1];
  request->range[2] = voxel_grid_range_[2];

  while (!voxel_grid_client_->wait_for_service(::std::chrono::seconds(1))) {
    if (!::rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the get "
                                 "voxel grid service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(),
                "Get voxel grid service not available, waiting again...");
  }
  auto result = voxel_grid_client_->async_send_request(
      request, ::std::bind(&Agent::VoxelGridResponseCallback, this,
                           ::std::placeholders::_1));
}

void Agent::MappingUtilVoxelGridCallback(
    const ::env_builder_msgs::msg::VoxelGridStamped::SharedPtr vg_msg) {
  ::env_builder_msgs::msg::VoxelGridStamped voxel_grid_stamped = *vg_msg;
  voxel_grid_mtx_.lock();
  voxel_grid_ =
      ::mapping_util::ConvertVGMsgToVGUtil(voxel_grid_stamped.voxel_grid);
  voxel_grid_mtx_.unlock();

  voxel_grid_ready_ = true;
  if (planner_verbose_) {
    RCLCPP_INFO(get_logger(), "Received the voxel grid");
  }
  if (publish_voxel_grid_) {
    PublishVoxelGrid();
  }
}

void Agent::GoalCallback(
    const ::geometry_msgs::msg::PointStamped::SharedPtr goal_msg) {
  goal_mtx_.lock();
  goal_curr_.resize(3);
  goal_curr_[0] = goal_msg->point.x;
  goal_curr_[1] = goal_msg->point.y;
  goal_curr_[2] = goal_msg->point.z;
  goal_mtx_.unlock();
}

void Agent::PublishVoxelGrid() {
  // copy voxel grid before reading and publishing it
  voxel_grid_mtx_.lock();
  ::voxel_grid_util::VoxelGrid vg_util = voxel_grid_;
  voxel_grid_mtx_.unlock();

  ::Eigen::Vector3d origin_vg = vg_util.GetOrigin();
  ::Eigen::Vector3i dim_vg = vg_util.GetDim();
  double vox_size = vg_util.GetVoxSize();

  ::pcl::PointCloud<::pcl::PointXYZ> cloud_occ;
  ::pcl::PointCloud<::pcl::PointXYZ> cloud_unk;
  ::pcl::PointCloud<::pcl::PointXYZ> cloud_free;

  // add obstacles points to point cloud
  for (int i = 0; i < int(dim_vg[0]); i++) {
    for (int j = 0; j < int(dim_vg[1]); j++) {
      for (int k = 0; k < int(dim_vg[2]); k++) {
        ::Eigen::Vector3i pt_i(i, j, k);
        ::pcl::PointXYZ pt;
        pt.x = i * vox_size + vox_size / 2 + origin_vg[0];
        pt.y = j * vox_size + vox_size / 2 + origin_vg[1];
        pt.z = k * vox_size + vox_size / 2 + origin_vg[2];
        if (vg_util.GetVoxelInt(pt_i) == ENV_BUILDER_OCC) {
          cloud_occ.points.push_back(pt);
        } else if (vg_util.GetVoxelInt(pt_i) >= ENV_BUILDER_FREE) {
          cloud_free.points.push_back(pt);
        } else if (vg_util.GetVoxelInt(pt_i) == ENV_BUILDER_UNK) {
          cloud_unk.points.push_back(pt);
        }
      }
    }
  }

  // create pc message
  auto pc_msg_ = ::std::make_shared<::sensor_msgs::msg::PointCloud2>();

  // publish occupied
  ::pcl::toROSMsg(cloud_occ, *pc_msg_);
  pc_msg_->header.frame_id = world_frame_;
  pc_msg_->header.stamp = now();
  voxel_grid_occ_pub_->publish(*pc_msg_);

  // publish free
  ::pcl::toROSMsg(cloud_free, *pc_msg_);
  pc_msg_->header.frame_id = world_frame_;
  pc_msg_->header.stamp = now();
  voxel_grid_free_pub_->publish(*pc_msg_);

  // publish unknown
  ::pcl::toROSMsg(cloud_unk, *pc_msg_);
  pc_msg_->header.frame_id = world_frame_;
  pc_msg_->header.stamp = now();
  voxel_grid_unk_pub_->publish(*pc_msg_);
}

void Agent::OnShutdown() {
  // save computation time to csv and display its mean, min, max and std dev
  ::std::string filename = "comp_time_sc_" + ::std::to_string(id_) + ".csv";
  SaveAndDisplayCompTime(comp_time_sc_, filename);
  filename = "comp_time_tasc_" + ::std::to_string(id_) + ".csv";
  SaveAndDisplayCompTime(comp_time_tasc_, filename);
  filename = "comp_time_opt_" + ::std::to_string(id_) + ".csv";
  SaveAndDisplayCompTime(comp_time_opt_, filename);
  filename = "comp_time_tot_" + ::std::to_string(id_) + ".csv";
  SaveAndDisplayCompTime(comp_time_tot_, filename);
  filename = "comp_time_tot_wall_" + ::std::to_string(id_) + ".csv";
  SaveAndDisplayCompTime(comp_time_tot_wall_, filename);
  filename = "comp_time_path_" + ::std::to_string(id_) + ".csv";
  SaveAndDisplayCompTime(comp_time_path_, filename);

  // save state history
  SaveStateHistory();

  // save and display communication latency
  SaveAndDisplayCommunicationLatency();
}
} // namespace multi_agent_planner
