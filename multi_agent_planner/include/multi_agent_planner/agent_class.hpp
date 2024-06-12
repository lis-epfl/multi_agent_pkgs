#ifndef MULTI_AGENT_PLANNER_AGENT_CLASS_H_
#define MULTI_AGENT_PLANNER_AGENT_CLASS_H_

#include "convex_decomp.hpp"
#include "decomp_ros_msgs/msg/polyhedron_array.hpp"
#include "decomp_ros_utils/data_ros_utils.h"
#include "env_builder_msgs/msg/voxel_grid.hpp"
#include "env_builder_msgs/msg/voxel_grid_stamped.hpp"
#include "env_builder_msgs/srv/get_voxel_grid.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "global_planner.hpp"
#include "gurobi_c++.h"
#include "mapping_util/map_builder.hpp"
#include "multi_agent_planner_msgs/msg/state.hpp"
#include "multi_agent_planner_msgs/msg/trajectory.hpp"
#include "path_tools.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "voxel_grid.hpp"

#include <decomp_geometry/polyhedron.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <jps_planner/distance_map_planner/distance_map_planner.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <mutex>
#include <pthread.h>

namespace multi_agent_planner {
class Agent : public ::rclcpp::Node {
public:
  // constructor
  Agent();

private:
  /*--------------------- class methods ---------------------*/
  // declare ros parameters
  void DeclareRosParameters();

  // initialize member variables from ros parameters
  void InitializeRosParameters();

  // initialize planner parameters
  void InitializePlannerParameters();

  // create the subscriber vector for the other agents
  void CreateTrajectorySubsriberVector();

  // trajectory subscriber callback function
  void TrajectoryOtherAgentsCallback(
      const ::multi_agent_planner_msgs::msg::Trajectory::SharedPtr &msg,
      const int &id);

  // create gurobi model
  GRBModel InitializeGurobi(GRBEnv &env);

  // create gurobi model
  void CreateGurobiModel();

  // compute model derivatives
  GRBLinExpr *ModelODE(GRBLinExpr *x_expr, ::std::vector<GRBVar> &u_i);

  // get path using distance map planner (DMP); it takes as input a starting
  // point, and a final point and a reference to the path output; returns false
  // if no path has been found
  bool GetPath(::std::vector<double> &start_arg,
               ::std::vector<double> &goal_arg,
               ::voxel_grid_util::VoxelGrid &voxel_grid,
               ::std::vector<::std::vector<double>> &path_out);

  // get path using our path finding library
  bool GetPathNew(::std::vector<double> &start_arg,
                  ::std::vector<double> &goal_arg,
                  ::voxel_grid_util::VoxelGrid &voxel_grid,
                  ::std::vector<::std::vector<double>> &path_out);

  // update path; run as a separate thread (one thread for the path and another
  // for solving the MIQP run in parallel and possibly at different frequencies)
  void UpdatePath();

  // plan trajectory by solving MPC/MIQP; run as a separate thread
  void TrajPlanningIteration();

  // generate reference trajectory for the MPC/MIQP
  void GenerateReferenceTrajectory();

  // generate safe corridor for only static obstacles
  void GenerateSafeCorridor();

  // generate time aware safe corridor for only static obstacles
  void GenerateTimeAwareSafeCorridor();

  // build the objective function/constraints and solve the MIQP/MPC and save
  // the result to member variables
  void SolveOptimizationProblem();

  // compute the yaw angle at this iteration using the reference that the drone
  // should be looking at the last reference point of the reference path for a P
  // yawing velocity controller.
  void ComputeYawAngle();

  // compute the attitude in the form of a quaternion from the acceleration
  // vector and the yawing angle
  ::Eigen::Quaterniond ComputeAttitude();

  // check if the trajectory points are close from the reference points so that
  // we can move along the trajectory
  void CheckReferenceTrajIncrement();

  // remove points that are the middle of two segments that have an acute angle
  // between them since they are redundant
  ::std::vector<::std::vector<double>>
  RemoveZigZagSegments(::std::vector<::std::vector<double>> path);

  // sample path using path_vel and n_hor_
  ::std::vector<std::vector<double>>
  SamplePath(::std::vector<::std::vector<double>> &path);

  // keep only the points that are in the free voxels for the reference
  // trajectory
  ::std::vector<::std::vector<double>>
  KeepOnlyFreeReference(::std::vector<::std::vector<double>> &traj_ref);

  // compute the path sampling velocity based on path and the voxel
  // grid/potential field; if the path is close to the obstacles, we wanna
  // sample it at slower speeds
  double ComputePathVelocity(::std::vector<::std::vector<double>> &path);

  // compute the velocity limit using the voxel/occupation value and the
  // distance to the start
  double GetVelocityLimit(double occ_val, double dist_start);

  // add hyperplane to a safe corridor
  ::std::vector<LinearConstraint3D>
  AddHyperplane(::std::vector<LinearConstraint3D> &poly_const_vec,
                Hyperplane3D &hp);

  // generate guroby polyhedron constraints from a polyhedron
  ::std::vector<GRBLinExpr>
  GetGurobiPolyhedronConstraints(LinearConstraint3D &poly, GRBLinExpr *x);

  // get intermediate goal
  ::std::vector<double>
  GetIntermediateGoal(::std::vector<double> &goal,
                      ::voxel_grid_util::VoxelGrid &voxel_grid);

  // clear the borders of the voxel grid
  void ClearBoundary(::voxel_grid_util::VoxelGrid &voxel_grid);

  // get distance squared between vectors
  double GetDistanceSquared(::std::vector<double> &p1,
                            ::std::vector<double> &p2);

  // check if a point is on the segment connecting 2 other points
  bool IsOnSegment(::std::vector<double> &pt, ::std::vector<double> &s_1,
                   ::std::vector<double> &s_2);

  // compute dot product of 2 vectors
  double DotProduct(::std::vector<double> &v_1, ::std::vector<double> &v_2);

  // save and display the mean, min and max computation time vector
  void SaveAndDisplayCompTime(::std::vector<double> &comp_time,
                              ::std::string &filename);

  // save traversed trajectory at the end of the node life (on shutdown)
  void SaveStateHistory();

  // save and display the communication latency statistics between agents
  void SaveAndDisplayCommunicationLatency();

  // publish the full generated trajectory for other agents
  void PublishTrajectoryFull();

  // publish the generated trajectory for rviz visualization
  void PublishTrajectory();

  // publish the generated path
  void PublishPath();

  // publish the generated path
  void PublishReferencePath();

  // publish the polyhedra seeds
  void PublishPolyhedraSeeds();

  // publish the polyhedra seeds
  void PublishPolyhedra();

  // publish current position as a marker and a transform
  void PublishCurrentPosition();

  // publish the traversed trajectory so far
  void PublishTrajectoryHistory();

  // publish the received voxel grid
  void PublishVoxelGrid();

  // get voxel grid service async call
  void GetVoxelGridAsync();

  // voxel grid service response callback
  void VoxelGridResponseCallback(
      ::rclcpp::Client<::env_builder_msgs::srv::GetVoxelGrid>::SharedFuture
          future);

  // voxel grid subsriber from mapping_util callback
  void MappingUtilVoxelGridCallback(
      const ::env_builder_msgs::msg::VoxelGridStamped::SharedPtr vg_msg);

  // current goal subscriber callback
  void
  GoalCallback(const ::geometry_msgs::msg::PointStamped::SharedPtr goal_msg);

  // function to execute on the shutdown of the node to save computation time
  // statistics
  void OnShutdown();

  /*-------------------- class variables --------------------*/
  /* voxel grid variables */
  // name of the service to get the voxel grid
  ::std::string get_grid_service_name_;
  // voxel_grid_range
  ::std::vector<double> voxel_grid_range_;
  // whether or not to publish the local voxel grid of the agent
  bool publish_voxel_grid_;
  // voxel grid update period in seconds
  double voxel_grid_update_period_;
  // voxel grid client
  ::rclcpp::Client<::env_builder_msgs::srv::GetVoxelGrid>::SharedPtr
      voxel_grid_client_;
  // voxel grid
  ::voxel_grid_util::VoxelGrid voxel_grid_;
  // timers for voxel service delay
  ::rclcpp::TimerBase::SharedPtr voxel_service_timer_;
  // whether to use the mapping_util or env_builder
  bool use_mapping_util_;

  /* subscribers and publishers */
  // publisher to publish the full generated trajectory (with velocity ...)
  ::rclcpp::Publisher<::multi_agent_planner_msgs::msg::Trajectory>::SharedPtr
      traj_full_pub_;
  // publisher to publish the generated trajectory
  ::rclcpp::Publisher<::nav_msgs::msg::Path>::SharedPtr traj_pub_;
  // publisher to publish the reference trajectory
  ::rclcpp::Publisher<::nav_msgs::msg::Path>::SharedPtr traj_ref_pub_;
  // publisher to publish the generated path
  ::rclcpp::Publisher<::nav_msgs::msg::Path>::SharedPtr path_pub_;
  // publisher to publish the seeds of the polyhedra
  ::rclcpp::Publisher<::sensor_msgs::msg::PointCloud2>::SharedPtr seeds_pub_;
  // publisher to publish the current position of the agent
  ::rclcpp::Publisher<::visualization_msgs::msg::Marker>::SharedPtr pos_pub_;
  // publisher to publish polyhedra
  ::rclcpp::Publisher<::decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr
      poly_pub_;
  // publisher to publish the traversed trajectory so far
  ::rclcpp::Publisher<::nav_msgs::msg::Path>::SharedPtr traj_hist_pub_;
  // publisher to publish occupied voxels
  ::rclcpp::Publisher<::sensor_msgs::msg::PointCloud2>::SharedPtr
      voxel_grid_occ_pub_;
  // publisher to publish free voxels
  ::rclcpp::Publisher<::sensor_msgs::msg::PointCloud2>::SharedPtr
      voxel_grid_free_pub_;
  // publisher to publish unknown voxels
  ::rclcpp::Publisher<::sensor_msgs::msg::PointCloud2>::SharedPtr
      voxel_grid_unk_pub_;
  // subscriber vector to get the trajectories of other agents
  ::std::vector<::rclcpp::Subscription<
      ::multi_agent_planner_msgs::msg::Trajectory>::SharedPtr>
      traj_other_sub_vec_;
  // subscriber to get the voxel grid in case we are using mapping_util
  ::rclcpp::Subscription<::env_builder_msgs::msg::VoxelGridStamped>::SharedPtr
      voxel_grid_sub_;
  // transform broadcaster to broadcast the position
  ::std::shared_ptr<::tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // subscriber to get the most recent goal
  ::rclcpp::Subscription<::geometry_msgs::msg::PointStamped>::SharedPtr
      goal_sub_;

  /* planner parameters */
  // topic prefix name that we add the id to it before publishing
  ::std::string topic_name_;
  // world frame
  ::std::string world_frame_;
  // total number of robot to avoid/listen to
  int n_rob_;
  // robot id
  int id_;
  // number of states: 6 for acceleration control and 9 for jerk control
  int n_x_;
  // number of control variables (3)
  int n_u_;
  // number of MPC steps/horizon
  int n_hor_;
  // time step MPC in seconds
  double dt_;
  // sampling path velocity
  double path_vel_;
  // minimum sampling path velocity
  double path_vel_min_;
  // maximum sampling path velocity
  double path_vel_max_;
  // distance sensitivity when computing the path_vel in GetVoxelVelocityLimit
  double sens_dist_;
  // potential sensitivity when computing the path_vel in GetVoxelVelocityLimit
  double sens_pot_;
  // sampling path velocity deceleration
  double path_vel_dec_;
  // number of reference trajectory points to keep at the next iteration
  int traj_ref_points_to_keep_;
  // whether to use rk4 or euler for integration
  bool rk4_;
  // how many time steps (MPC) between planning iterations
  int step_plan_;
  // distance used to see if the reference path should move forward: if an MPC
  // point is within this distance from its reference point, or if the first
  // MPC point is within this distance from any reference point, the reference
  // path is moved forward
  double thresh_dist_;
  // number of polyhedra to consider at each planning iteration
  int poly_hor_;
  // number of expansion iterations to generate the convex polyhedron
  int n_it_decomp_;
  // boolean to determine which polyhedron generation method to use: if true
  // use convex_decomp_util (my method); if false use decomp_util (liu's
  // method)
  bool use_cvx_;
  // (only if use_cvx_ is true) use the original polyhedron decomposition
  // method or the new shape aware one (the new one may not be as good)
  bool use_cvx_new_;
  // drone safety radius (a value in the ellipse function)
  double drone_radius_;
  // drone z offset for downwash (b value in the ellipse function)
  double drone_z_offset_;
  // obstacles inflation (in number of voxels) for path finding (this is in
  // addition to the inflation to account for the drone radius); used to allow
  // for better polyhedron generation in tight corridors in case there is no
  // adaptation for the grid resolution in tight spaces
  double path_infl_dist_;
  // communication latency in seconds (added artificially)
  double com_latency_;
  // MPC control weight
  double r_u_;
  // MPC trajectory error weight (except last point)
  ::std::vector<double> r_x_;
  // MPC last point error weight
  ::std::vector<double> r_n_;
  // MPC maximum velocity
  double max_vel_;
  // MPC maximum acceleration in z direction
  double max_acc_z_;
  // MPC minimum acceleration in z direction
  double min_acc_z_;
  // MPC maximum acceleration in x,y directions
  double max_acc_xy_;
  // MPC minimum acceleration in x,y directions
  double min_acc_xy_;
  // MPC maximum jerk
  double max_jerk_;
  // mass of the drone
  double mass_;
  // drag coefficients
  ::std::vector<double> drag_coeff_;
  // print info about the planner (comp time ...)
  bool planner_verbose_;
  // save statistics about computation time and state history
  bool save_stats_;

  /* yaw control variables */
  // current yaw angle
  double yaw_;
  // index of the point of the reference trajectory that we want to look at with
  // the yaw
  int yaw_idx_;
  // P velocity control of the yaw angle
  double k_p_yaw_;

  /* path planner params */
  // distance map planner search radius
  double dmp_search_rad_;
  // number of iteration for the dmp planner
  double dmp_n_it_;
  // path planning path planning period in seconds
  double path_planning_period_;
  // whether to reset the path planning if there is an obstacle between the
  // reference trajectory and the trajectory
  bool reset_path_ = false;

  /* create thread variables (if we don't use member variables and instead we
   * use local variables in the constructor for the threads, we get an error)
   */
  // path planning thread
  ::std::thread path_planning_thread_;
  // trajectory planning thread
  ::std::thread traj_planning_thread_;

  /* gurobi variables */
  // environment + model
  GRBEnv env_;
  GRBModel model_;
  // if true, display gurobi optimization results
  bool gurobi_verbose_;
  // state variables
  ::std::vector<::std::vector<GRBVar>> x_grb_;
  // control variables
  ::std::vector<::std::vector<GRBVar>> u_grb_;
  // binary variables for polyhedron contraints
  ::std::vector<::std::vector<GRBVar>> b_grb_;
  // optimization objective
  GRBQuadExpr obj_grb_ = 0;
  // polyhedra constraints added in gurobi model
  ::std::vector<GRBGenConstr> poly_constr_grb_;
  // at least one polyhedron constraint for each segment
  ::std::vector<GRBConstr> at_least_1_poly_constr_grb_;
  // lower bound on the control
  ::std::vector<double> u_lb_;
  // upper bound on the control
  ::std::vector<double> u_ub_;
  // lower bound on the states
  ::std::vector<double> x_lb_;
  // upper bound on the states
  ::std::vector<double> x_ub_;

  /* trajectory planning variable */
  // initial state of drone; for now fixed as a config but it should be taken
  // from external simulator or real drone
  ::std::vector<double> state_ini_;
  // current considered state for optimization which includes position,
  // velocity, and acceleration concatenated; for now it will be the second
  // point in the generated trajectory i.e. the expected point that the agent
  // will be at at the next iteration because at each iteration, we plan
  // starting from where the agent is gonna be at at the next iteration; we
  // should also have the option to get it from an external simulator/real
  // drone, by taking the current value of the simulator/real drone and
  // projecting it to the next iteration
  ::std::vector<double> state_curr_;
  // current goal
  ::std::vector<double> goal_curr_;
  // current generated path
  ::std::vector<::std::vector<double>> path_curr_;
  // current reference trajectory for N discrete points (the first point
  // is fixed and does not have a reference)
  ::std::vector<::std::vector<double>> traj_ref_curr_;
  // starting point for the sampling of the reference trajectory; it can be
  // updated using the path progress concept
  ::std::vector<double> starting_point_;
  // current generated trajectory (concatenation of the following: position,
  // velocity, acceleration)
  ::std::vector<::std::vector<double>> traj_curr_;
  // current generated control/command (jerk)
  ::std::vector<::std::vector<double>> control_curr_;
  // received trajectories of other drones
  ::std::vector<::multi_agent_planner_msgs::msg::Trajectory> traj_other_agents_;
  // seeds used for the generation of the polyhedra
  ::std::vector<::std::vector<double>> poly_seeds_;
  // polyhedra for visualization without other agents
  vec_E<Polyhedron3D> poly_vec_;
  // polyhedra constraints without other agents (only for static obstacles)
  ::std::vector<LinearConstraint3D> poly_const_vec_;
  // polyhedra constraints accounting for static obstacles and other agents
  ::std::vector<::std::vector<LinearConstraint3D>> poly_const_final_vec_;
  // bool to indicate that we received the first voxel grid
  bool voxel_grid_ready_ = false;
  // bool to indicate that we generated the first path
  bool path_ready_ = false;
  // bool to indicate if we should increment the reference trajectory
  bool increment_traj_ref_ = false;
  // bool to indicate if the gurobi optimization has failed
  bool optimization_failed_;
  // vector to save the polyhedra that were in the optimization; it is resized
  // to size poly_hor_ and if ith idx is true, it means we are using the ith
  // poly
  ::std::vector<bool> poly_used_idx_;
  // if true remove corners when doing the RemoveZigZagSegments function:  When
  // close to the obstacles, we will cut corners and allow digonal motion which
  // is not ideal for Safe Corridor generation in edge cases
  bool remove_corners_;

  /* mutex variables for memory management */
  // mutex for voxel grid
  ::std::mutex voxel_grid_mtx_;
  // mutex for path planning (JPS/DMP set and the MIQP read)
  ::std::mutex path_mtx_;
  // mutex for goal vector (JPS/DMP read and MIQP set)
  ::std::mutex goal_mtx_;
  // mutex for accessing the reference trajectory
  ::std::mutex traj_ref_mtx_;
  // mutex for updating the state of the drone
  ::std::mutex state_mtx_;
  // mutex for accessing trajectories of other agents
  ::std::vector<::std::mutex> traj_other_mtx_;

  /* computation time variables in milliseconds in cpu time (unless specified
   * otherwise) */
  // path planning computation time
  ::std::vector<double> comp_time_path_;
  // safe corridor building computation time
  ::std::vector<double> comp_time_sc_;
  // time aware safe corridor building computation time
  ::std::vector<double> comp_time_tasc_;
  // optimization computation time
  ::std::vector<double> comp_time_opt_;
  // total iteration computation time (safe corridor + optimization)
  ::std::vector<double> comp_time_tot_;
  // total iteration computation time wall time (safe corridor + optimization)
  ::std::vector<double> comp_time_tot_wall_;

  // state history variable
  ::std::vector<::std::vector<double>> state_hist_;
  // state history stamp variable (the wall time of each state of state_hist_)
  ::std::vector<double> state_hist_stamp_;

  // communication latency history for the full trajectory communication in
  // milliseconds
  ::std::vector<::std::vector<double>> com_latency_ms_;
};
} // namespace multi_agent_planner

#endif // MULTI_AGENT_PLANNER_AGENT_CLASS_H_
