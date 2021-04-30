/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 * Copyright (c) 2020 LAAS/CNRS
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 * Modified by: Phani Teja Singamaneni
 *********************************************************************/

#define THROTTLE_RATE 1.0 // seconds
#include <hateb_local_planner/optimal_planner.h>
#include <map>
#include <memory>
#include <limits>
#include <fstream>
#include <iostream>


namespace hateb_local_planner
{

// ============== Implementation ===================

TebOptimalPlanner::TebOptimalPlanner() : cfg_(NULL), obstacles_(NULL), via_points_(NULL), cost_(HUGE_VAL), prefer_rotdir_(RotType::none),
                                         robot_model_(new PointRobotFootprint()), human_model_(new CircularRobotFootprint()), initialized_(false), optimized_(false)
{
}

TebOptimalPlanner::TebOptimalPlanner(const HATebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points, CircularRobotFootprintPtr human_model,
    const std::map<uint64_t, ViaPointContainer> *humans_via_points_map)
{
  initialize(cfg, obstacles, robot_model, visual, via_points,  human_model, humans_via_points_map);
}

TebOptimalPlanner::~TebOptimalPlanner()
{
  clearGraph();
  // free dynamically allocated memory
  //if (optimizer_)
  //  g2o::Factory::destroy();
  //g2o::OptimizationAlgorithmFactory::destroy();
  //g2o::HyperGraphActionLibrary::destroy();
}

void TebOptimalPlanner::initialize(const HATebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points, CircularRobotFootprintPtr human_model, const std::map<uint64_t, ViaPointContainer> *humans_via_points_map)
{
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();

  cfg_ = &cfg;
  obstacles_ = obstacles;
  robot_model_ = robot_model;
  human_model_ = human_model;
  via_points_ = via_points;
  humans_via_points_map_ = humans_via_points_map;
  cost_ = HUGE_VAL;
  prefer_rotdir_ = RotType::none;
  setVisualization(visual);

  vel_start_.first = true;
  vel_start_.second.linear.x = 0;
  vel_start_.second.linear.y = 0;
  vel_start_.second.angular.z = 0;

  vel_goal_.first = true;
  vel_goal_.second.linear.x = 0;
  vel_goal_.second.linear.y = 0;
  vel_goal_.second.angular.z = 0;

  robot_radius_ = robot_model_->getCircumscribedRadius();
  // std::cout << "robot_radius " <<robot_radius_<< '\n';
  human_radius_ = human_model_->getCircumscribedRadius();
  // std::cout << "human_model_ " << human_radius_<< '\n';

  initialized_ = true;
  isMode = 0;
}


void TebOptimalPlanner::setVisualization(TebVisualizationPtr visualization)
{
  visualization_ = visualization;
}

void TebOptimalPlanner::visualize()
{
  if (!visualization_)
    return;

  double fp_size= 0.0;
  fp_size = teb_.sizePoses();

  for (auto &human_teb_kv : humans_tebs_map_) {
      auto &human_teb = human_teb_kv.second;
      fp_size = teb_.sizePoses() > human_teb.sizePoses() ? teb_.sizePoses() : human_teb.sizePoses();
    }
  visualization_->publishLocalPlanAndPoses(teb_, *robot_model_, fp_size);
  visualization_->publishHumanLocalPlansAndPoses(humans_tebs_map_, *human_model_, fp_size);

  if (teb_.sizePoses() > 0)
    visualization_->publishRobotFootprintModel(teb_.Pose(0), *robot_model_);

  if (cfg_->trajectory.publish_feedback)
    visualization_->publishFeedbackMessage(*this, *obstacles_);

}


/*
 * registers custom vertices and edges in g2o framework
 */
void TebOptimalPlanner::registerG2OTypes()
{
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
  factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<VertexTimeDiff>);

  factory->registerType("EDGE_TIME_OPTIMAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimal>);
  factory->registerType("EDGE_SHORTEST_PATH", new g2o::HyperGraphElementCreator<EdgeShortestPath>);
  factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>);
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>);
  factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>);
  factory->registerType("EDGE_ACCELERATION_START", new g2o::HyperGraphElementCreator<EdgeAccelerationStart>);
  factory->registerType("EDGE_ACCELERATION_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>);
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>);
  factory->registerType("EDGE_KINEMATICS_CARLIKE", new g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>);
  factory->registerType("EDGE_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeObstacle>);
  factory->registerType("EDGE_INFLATED_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeInflatedObstacle>);
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeDynamicObstacle>);
  factory->registerType("EDGE_VIA_POINT", new g2o::HyperGraphElementCreator<EdgeViaPoint>);
  factory->registerType("EDGE_PREFER_ROTDIR", new g2o::HyperGraphElementCreator<EdgePreferRotDir>);

  //Humans
  factory->registerType("EDGE_VELOCITY_HUMAN", new g2o::HyperGraphElementCreator<EdgeVelocityHuman>);
  factory->registerType("EDGE_VELOCITY_HOLONOMIC_HUMAN", new g2o::HyperGraphElementCreator<EdgeVelocityHolonomicHuman>);
  factory->registerType("EDGE_ACCELERATION_HUMAN", new g2o::HyperGraphElementCreator<EdgeAccelerationHuman>);
  factory->registerType("EDGE_ACCELERATION_HUMAN_START", new g2o::HyperGraphElementCreator<EdgeAccelerationHumanStart>);
  factory->registerType("EDGE_ACCELERATION_HUMAN_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationHumanGoal>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_HUMAN", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicHuman>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_HUMAN_START", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicHumanStart>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_HUMAN_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicHumanGoal>);
  factory->registerType("EDGE_HUMAN_ROBOT_SAFETY", new g2o::HyperGraphElementCreator<EdgeHumanRobotSafety>);
  factory->registerType("EDGE_HUMAN_HUMAN_SAFETY", new g2o::HyperGraphElementCreator<EdgeHumanHumanSafety>);
  factory->registerType("EDGE_HUMAN_ROBOT_TTC", new g2o::HyperGraphElementCreator<EdgeHumanRobotTTC>);
  factory->registerType("EDGE_HUMAN_ROBOT_TTCplus", new g2o::HyperGraphElementCreator<EdgeHumanRobotTTCplus>);
  factory->registerType("EDGE_HUMAN_ROBOT_REL_VELOCITy", new g2o::HyperGraphElementCreator<EdgeHumanRobotRelVelocity>);
  return;
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
boost::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
  static boost::once_flag flag = BOOST_ONCE_INIT;
  boost::call_once(&registerG2OTypes, flag);

  // allocating the optimizer
  boost::shared_ptr<g2o::SparseOptimizer> optimizer = boost::make_shared<g2o::SparseOptimizer>();
  std::unique_ptr<TEBLinearSolver> linear_solver(new TEBLinearSolver()); // see typedef in optimization.h
  linear_solver->setBlockOrdering(true);
  std::unique_ptr<TEBBlockSolver> block_solver(new TEBBlockSolver(std::move(linear_solver)));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  optimizer->setAlgorithm(solver);

  optimizer->initMultiThreading(); // required for >Eigen 3.1

  return optimizer;
}

bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                                    double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost, hateb_local_planner::OptimizationCostArray *op_costs){

  optimizeTEB(iterations_innerloop,iterations_outerloop,compute_cost_afterwards,obst_cost_scale,viapoint_cost_scale,alternative_time_cost,op_costs,cfg_->trajectory.dt_ref,cfg_->trajectory.dt_hysteresis);

}

bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                                    double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost, hateb_local_planner::OptimizationCostArray *op_costs, double dt_ref, double dt_hyst)
{
  if (cfg_->optim.optimization_activate==false)
    return false;

  bool success = false;
  optimized_ = false;

  double weight_multiplier = 1.0;

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
  //                (which leads to better results in terms of x-y-t homotopy planning).
  //                 however, we have not tested this mode intensively yet, so we keep
  //                 the legacy fast mode as default until we finish our tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;

  for(int i=0; i<iterations_outerloop; ++i)
  {
    if (cfg_->trajectory.teb_autosize)
    {
      //teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples);
      // teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);
      teb_.autoResize(dt_ref, dt_hyst,
                      cfg_->trajectory.min_samples);
        for (auto &human_teb_kv : humans_tebs_map_)
          // human_teb_kv.second.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);
          human_teb_kv.second.autoResize(dt_ref,
                                         dt_hyst,
                                         cfg_->trajectory.min_samples);
    }

    success = buildGraph(weight_multiplier);
    if (!success)
    {
        clearGraph();
        return false;
    }
    success = optimizeGraph(iterations_innerloop, false);
    if (!success)
    {
        clearGraph();
        return false;
    }
    optimized_ = true;

    if (compute_cost_afterwards && i==iterations_outerloop-1) // compute cost vec only in the last iteration
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost, op_costs);

    clearGraph();

    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  return true;
}

void TebOptimalPlanner::setVelocityStart(const geometry_msgs::Twist& vel_start)
{
  vel_start_.first = true;
  vel_start_.second.linear.x = vel_start.linear.x;
  vel_start_.second.linear.y = vel_start.linear.y;
  vel_start_.second.angular.z = vel_start.angular.z;
}

void TebOptimalPlanner::setVelocityGoal(const geometry_msgs::Twist& vel_goal)
{
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}

bool TebOptimalPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel, bool free_goal_vel, const HumanPlanVelMap *initial_human_plan_vel_map, hateb_local_planner::OptimizationCostArray *op_costs, double dt_ref, double dt_hyst)
{
  // std::cout << "I am in the traj plan function" << '\n';
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  auto prep_start_time = ros::Time::now();
  if (!teb_.isInit())
  {
    // init trajectory
    // teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion,cfg_->trajectory.teb_init_skip_dist);
    teb_.initTEBtoGoal(initial_plan, dt_ref, true,
                       cfg_->trajectory.min_samples,
                       cfg_->trajectory.teb_init_skip_dist);
  }
  else if (cfg_->optim.disable_warm_start){
    teb_.clearTimedElasticBand();
    // teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion,cfg_->trajectory.teb_init_skip_dist);
    teb_.initTEBtoGoal(initial_plan, dt_ref, true,
                       cfg_->trajectory.min_samples,
                       cfg_->trajectory.teb_init_skip_dist);

  }
  else // warm start
  {
    PoseSE2 start_(initial_plan.front().pose);
    PoseSE2 goal_(initial_plan.back().pose);
    if (teb_.sizePoses()>0
        && (goal_.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal_.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular)
    { // actual warm start!
      // std::cout << teb_.sizePoses() << '\n';
      teb_.updateAndPruneTEB(start_, goal_, cfg_->trajectory.min_samples); // update TEB
    }
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      // teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, true, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
      teb_.initTEBtoGoal(initial_plan, dt_ref, true,
                         cfg_->trajectory.min_samples,
                         cfg_->trajectory.teb_init_skip_dist);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
  auto prep_time = ros::Time::now() - prep_start_time;

  auto human_prep_time_start = ros::Time::now();
  humans_vel_start_.clear();
  humans_vel_goal_.clear();
  human_nominal_vels.clear();
  isMode = 0;

  current_human_robot_min_dist = std::numeric_limits<double>::max();

  switch (cfg_->planning_mode) {
  case 0:
    humans_tebs_map_.clear();
    break;
  case 1: {
    auto itr = humans_tebs_map_.begin();
    while (itr != humans_tebs_map_.end()) {
      if (initial_human_plan_vel_map->find(itr->first) ==
          initial_human_plan_vel_map->end())
        itr = humans_tebs_map_.erase(itr);
      else
        ++itr;
    }

    auto &rp = initial_plan.front().pose.position;

    for (auto &initial_human_plan_vel_kv : *initial_human_plan_vel_map) {
      auto &human_id = initial_human_plan_vel_kv.first;
      auto &initial_human_plan = initial_human_plan_vel_kv.second.plan;
      human_nominal_vels.push_back(initial_human_plan_vel_kv.second.nominal_vel);
      isMode = initial_human_plan_vel_kv.second.isMode;
      // erase human-teb if human plan is empty
      if (initial_human_plan.empty()) {
        auto itr = humans_tebs_map_.find(human_id);
        if (itr != humans_tebs_map_.end()) {
          ROS_DEBUG("New plan: new human plan is empty. Removing human trajectories.");
          humans_tebs_map_.erase(itr);
        }
        continue;
      }

      auto &hp = initial_human_plan.front().pose.position;
      auto dist = std::hypot(rp.x - hp.x, rp.y - hp.y);
      if (dist < current_human_robot_min_dist) {
        current_human_robot_min_dist = dist;
      }

      if (humans_tebs_map_.find(human_id) == humans_tebs_map_.end())
      {
        // create new human-teb for new human
        humans_tebs_map_[human_id] = TimedElasticBand();
        // humans_tebs_map_[human_id].initTrajectoryToGoal(initial_human_plan, cfg_->human.max_vel_x, true, cfg_->trajectory.human_min_samples, cfg_->trajectory.allow_init_with_backwards_motion, cfg_->trajectory.teb_init_skip_dist);
        humans_tebs_map_[human_id].initTEBtoGoal(
            initial_human_plan, dt_ref, true,
            cfg_->trajectory.human_min_samples,
            cfg_->trajectory.teb_init_skip_dist);
      }
      else if (cfg_->optim.disable_warm_start)
      {
        auto &human_teb = humans_tebs_map_[human_id];
        human_teb.clearTimedElasticBand();
        // human_teb.initTrajectoryToGoal(initial_human_plan, cfg_->human.max_vel_x, true, cfg_->trajectory.human_min_samples, cfg_->trajectory.allow_init_with_backwards_motion, cfg_->trajectory.teb_init_skip_dist);
        human_teb.initTEBtoGoal(initial_human_plan, dt_ref,
                                true, cfg_->trajectory.human_min_samples,
                                cfg_->trajectory.teb_init_skip_dist);
      }

      else
      {
      // modify human-teb for existing human
        PoseSE2 human_start_(initial_human_plan.front().pose);
        PoseSE2 human_goal_(initial_human_plan.back().pose);
        auto &human_teb = humans_tebs_map_[human_id];
        if (human_teb.sizePoses() > 0 && (human_goal_.position() - human_teb.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist)
        human_teb.updateAndPruneTEB(human_start_, human_goal_, cfg_->trajectory.human_min_samples);
        else
        {
        ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitializing human trajectories.");
        human_teb.clearTimedElasticBand();
        // human_teb.initTrajectoryToGoal(initial_human_plan, cfg_->human.max_vel_x, true, cfg_->trajectory.human_min_samples, false, cfg_->trajectory.teb_init_skip_dist);
        human_teb.initTEBtoGoal(initial_human_plan, dt_ref,
                                true, cfg_->trajectory.human_min_samples,
                                cfg_->trajectory.teb_init_skip_dist);
        }
      }
      // give start velocity for humans
      std::pair<bool, geometry_msgs::Twist> human_start_vel;
      human_start_vel.first = true;
      human_start_vel.second.linear.x = initial_human_plan_vel_kv.second.start_vel.linear.x;
      human_start_vel.second.linear.y = initial_human_plan_vel_kv.second.start_vel.linear.y;
      human_start_vel.second.angular.z = initial_human_plan_vel_kv.second.start_vel.angular.z;
      humans_vel_start_[human_id] = human_start_vel;

      // do not set goal velocity for humans
      std::pair<bool, geometry_msgs::Twist> human_goal_vel;
      human_goal_vel.first = false;
      // human_goal_vel.first = true;
      // human_goal_vel.second.coeffRef(0) =
      //     initial_human_plan_vel_kv.second.goal_vel.linear.x;
      // human_goal_vel.second.coeffRef(1) =
      //     initial_human_plan_vel_kv.second.goal_vel.angular.z;
      // humans_vel_goal_[human_id] = human_goal_vel;
    }
    break;
  }
  case 2: {
    if (initial_human_plan_vel_map->size() == 1)
    {
      auto &approach_plan = initial_human_plan_vel_map->begin()->second.plan;
      if (approach_plan.size() == 1)
      {
        approach_pose_ = approach_plan.front();
        // modify robot global plan
      }
      else
      {
        ROS_INFO("empty pose of the human for approaching");
        // set approach_pose_ same as the current robot pose
        approach_pose_ = initial_plan.front();
      }
    }
    else
    {
      ROS_INFO("no or multiple humans for approaching %d", initial_human_plan_vel_map->size());
      // set approach_pose_ same as the current robot pose
      approach_pose_ = initial_plan.front();
    }
    break;
  }
  default:
    humans_tebs_map_.clear();
  }
  auto human_prep_time = ros::Time::now() - human_prep_time_start;

  // now optimize
  auto opt_start_time = ros::Time::now();
  bool teb_opt_result = optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations, true, 1.0, 1.0, false, op_costs, dt_ref, dt_hyst);

  if (op_costs) {
    hateb_local_planner::OptimizationCost op_cost;
    op_cost.type = hateb_local_planner::OptimizationCost::HUMAN_ROBOT_MIN_DIST;
    op_cost.cost = current_human_robot_min_dist;
    op_costs->costs.push_back(op_cost);
  }

  auto opt_time = ros::Time::now() - opt_start_time;

  auto total_time = ros::Time::now() - prep_start_time;

  ROS_DEBUG_STREAM_COND(total_time.toSec() > 0.1,
                        "\nteb optimal plan times:\n"
                            << "\ttotal plan time                "
                            << std::to_string(total_time.toSec()) << "\n"
                            << "\toptimizatoin preparation time  "
                            << std::to_string(prep_time.toSec()) << "\n"
                            << "\thuman preparation time         "
                            << std::to_string(prep_time.toSec()) << "\n"
                            << "\tteb optimize time              "
                            << std::to_string(opt_time.toSec())
                            << "\n-------------------------");

  return teb_opt_result;
}


bool TebOptimalPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel, hateb_local_planner::OptimizationCostArray *op_costs, double dt_ref, double dt_hyst)
{
  auto start_time = ros::Time::now();
  PoseSE2 start_(start);
  PoseSE2 goal_(goal);
  geometry_msgs::Twist *zero_vel;
  const geometry_msgs::Twist *vel = start_vel ? start_vel : zero_vel;
  auto pre_plan_time = ros::Time::now() - start_time;
  return plan(start_, goal_, vel, free_goal_vel, pre_plan_time.toSec(), op_costs, dt_ref, dt_hyst);
}

bool TebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel, double pre_plan_time, hateb_local_planner::OptimizationCostArray *op_costs, double dt_ref, double dt_hyst)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  auto prep_start_time = ros::Time::now();
  if (!teb_.isInit())
  {
    // init trajectory
    // teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion); // 0 intermediate samples, but dt=1 -> autoResize will add more samples before calling first optimization
    teb_.initTEBtoGoal(start, goal, 0, 1,
                       cfg_->trajectory.min_samples);
  }
  else // warm start
  {
    if (teb_.sizePoses() > 0
        && (goal.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
      teb_.updateAndPruneTEB(start, goal, cfg_->trajectory.min_samples);
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      // teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
      teb_.initTEBtoGoal(start, goal, 0, 1, cfg_->trajectory.min_samples);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
    auto prep_time = ros::Time::now() - prep_start_time;
  // now optimize

  auto opt_start_time = ros::Time::now();
  bool teb_opt_result = optimizeTEB(cfg_->optim.no_inner_iterations,
                                    cfg_->optim.no_outer_iterations,true, 1.0, 1.0, false, op_costs, dt_ref,dt_hyst);
  auto opt_time = ros::Time::now() - opt_start_time;

  auto total_time = ros::Time::now() - prep_start_time;
  ROS_INFO_STREAM_COND(
      (total_time.toSec() + pre_plan_time) > 0.05,
      "\nteb optimal plan times:\n"
          << "\ttotal plan time                "
          << std::to_string(total_time.toSec() + pre_plan_time) << "\n"
          << "\tpre-plan time                  "
          << std::to_string(pre_plan_time) << "\n"
          << "\toptimizatoin preparation time  "
          << std::to_string(prep_time.toSec()) << "\n"
          << "\tteb optimize time              "
          << std::to_string(opt_time.toSec()) << "\n-------------------------");

 return teb_opt_result;
}


bool TebOptimalPlanner::buildGraph(double weight_multiplier)
{
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  // add TEB vertices
  AddTEBVertices();

  // add Edges (local cost functions)
  if (cfg_->obstacles.legacy_obstacle_association)
    AddEdgesObstaclesLegacy(weight_multiplier);
  else
    AddEdgesObstacles(weight_multiplier);

  if (cfg_->obstacles.include_dynamic_obstacles)
    AddEdgesDynamicObstacles();

  AddEdgesViaPoints();

  AddEdgesVelocity();

  AddEdgesAcceleration();

  AddEdgesTimeOptimal();

  AddEdgesShortestPath();

  if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0)
    AddEdgesKinematicsDiffDrive(); // we have a differential drive robot
  else
    AddEdgesKinematicsCarlike(); // we have a carlike robot since the turning radius is bounded from below.


  AddEdgesPreferRotDir();

  switch (cfg_->planning_mode) {
  case 0:
    break;
  case 1:
    AddEdgesObstaclesForHumans();
    // AddEdgesDynamicObstaclesForHumans();

    AddEdgesViaPointsForHumans();

    AddEdgesVelocityForHumans();
    AddEdgesAccelerationForHumans();

    AddEdgesTimeOptimalForHumans();

    AddEdgesKinematicsDiffDriveForHumans();
    // AddEdgesKinematicsCarlikeForHumans();

    if (cfg_->hateb.use_human_robot_safety_c) {
      AddEdgesHumanRobotSafety();
    }

    if (cfg_->hateb.use_human_human_safety_c) {
      AddEdgesHumanHumanSafety();
    }

    if (cfg_->hateb.use_human_robot_ttc_c) {
      AddEdgesHumanRobotTTC();
    }

    if (cfg_->hateb.use_human_robot_ttcplus_c) {
      AddEdgesHumanRobotTTCplus();
    }
    if (cfg_->hateb.use_human_robot_rel_vel_c) {
      AddEdgesHumanRobotRelVelocity();
    }
    if (cfg_->hateb.use_human_robot_visi_c){
        AddEdgesHumanRobotVisibility();
    }
    break;
  case 2:
    AddVertexEdgesApproach();
    break;
  default:
    break;
  }

  return true;
}

bool TebOptimalPlanner::optimizeGraph(int no_iterations,bool clear_after)
{
  if (cfg_->robot.max_vel_x<0.01)
  {
    ROS_WARN("optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
    if (clear_after) clearGraph();
    return false;
  }

  if (!teb_.isInit() || (int)teb_.sizePoses() < cfg_->trajectory.min_samples)
  {
    ROS_WARN("optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
    if (clear_after) clearGraph();
    return false;
  }

  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_iterations);

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if(!iter)
  {
	ROS_ERROR("optimizeGraph(): Optimization failed! iter=%i", iter);
	return false;
  }

  if (clear_after) clearGraph();

  return true;
}

void TebOptimalPlanner::clearGraph()
{
  // clear optimizer states
  //optimizer.edges().clear(); // optimizer.clear deletes edges!!! Therefore do not run optimizer.edges().clear()
  if (optimizer_){
   //optimizer.edges().clear(); // optimizer.clear deletes edges!!! Therefore do not run optimizer.edges().clear()
    optimizer_->vertices().clear();  // neccessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();

  }
}



void TebOptimalPlanner::AddTEBVertices()
{
  // add vertices to graph
  ROS_DEBUG_COND(cfg_->optim.optimization_verbose, "Adding TEB vertices ...");
  unsigned int id_counter = 0; // used for vertices ids
  for (int i=0; i<teb_.sizePoses(); ++i)
  {
    teb_.PoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(teb_.PoseVertex(i));
    if (teb_.sizeTimeDiffs()!=0 && i<teb_.sizeTimeDiffs())
    {
      teb_.TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.TimeDiffVertex(i));
    }
  }

  switch (cfg_->planning_mode) {
  case 0:
    break;
  case 1: {
    for (auto &human_teb_kv : humans_tebs_map_) {
      auto &human_teb = human_teb_kv.second;
      // std::cout << "HumanTebsizes " << human_teb.sizePoses() << '\n';
      for (int i = 0; i < human_teb.sizePoses(); ++i) {
        human_teb.PoseVertex(i)->setId(id_counter++);
        optimizer_->addVertex(human_teb.PoseVertex(i));
        if (teb_.sizeTimeDiffs() != 0 && i < human_teb.sizeTimeDiffs()) {
          human_teb.TimeDiffVertex(i)->setId(id_counter++);
          optimizer_->addVertex(human_teb.TimeDiffVertex(i));
        }
      }
    }
    break;
  }
  case 2: {
    PoseSE2 approach_pose_se2(approach_pose_.pose);
    approach_pose_vertex = new VertexPose(approach_pose_se2, true);
    approach_pose_vertex->setId(id_counter++);
    optimizer_->addVertex(approach_pose_vertex);
    break;
  }
  default:
    break;
  }
}

void TebOptimalPlanner::AddEdgesObstacles(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr )
    return; // if weight equals zero skip adding edges!

   bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;

  // iterate all teb points (skip first and last)
  for (int i=1; i < teb_.sizePoses()-1; ++i)
  {
      double left_min_dist = std::numeric_limits<double>::max();
      double right_min_dist = std::numeric_limits<double>::max();
      Obstacle* left_obstacle = nullptr;
      Obstacle* right_obstacle = nullptr;

      std::vector<Obstacle*> relevant_obstacles;

      const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();

      // iterate obstacles
      for (const ObstaclePtr& obst : *obstacles_)
      {
        // we handle dynamic obstacles differently below
        if(cfg_->obstacles.include_dynamic_obstacles && obst->isDynamic())
          continue;

          // calculate distance to robot model
          double dist = robot_model_->calculateDistance(teb_.Pose(i), obst.get());

          // force considering obstacle if really close to the current pose
        if (dist < cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor)
          {
              relevant_obstacles.push_back(obst.get());
              continue;
          }
          // cut-off distance
          if (dist > cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_cutoff_factor)
            continue;

          // determine side (left or right) and assign obstacle if closer than the previous one
          if (cross2d(pose_orient, obst->getCentroid()) > 0) // left
          {
              if (dist < left_min_dist)
              {
                  left_min_dist = dist;
                  left_obstacle = obst.get();
              }
          }
          else
          {
              if (dist < right_min_dist)
              {
                  right_min_dist = dist;
                  right_obstacle = obst.get();
              }
          }
      }

      // create obstacle edges
      if (left_obstacle)
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
                dist_bandpt_obst->setVertex(0,teb_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information_inflated);
                dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), left_obstacle);
                optimizer_->addEdge(dist_bandpt_obst);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
                dist_bandpt_obst->setVertex(0,teb_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information);
                dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), left_obstacle);
                optimizer_->addEdge(dist_bandpt_obst);
            }
      }

      if (right_obstacle)
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
                dist_bandpt_obst->setVertex(0,teb_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information_inflated);
                dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), right_obstacle);
                optimizer_->addEdge(dist_bandpt_obst);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
                dist_bandpt_obst->setVertex(0,teb_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information);
                dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), right_obstacle);
                optimizer_->addEdge(dist_bandpt_obst);
            }
      }

      for (const Obstacle* obst : relevant_obstacles)
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
                dist_bandpt_obst->setVertex(0,teb_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information_inflated);
                dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst);
                optimizer_->addEdge(dist_bandpt_obst);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
                dist_bandpt_obst->setVertex(0,teb_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information);
                dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst);
                optimizer_->addEdge(dist_bandpt_obst);
            }
      }
  }

}


void TebOptimalPlanner::AddEdgesObstaclesLegacy(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;

  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (cfg_->obstacles.include_dynamic_obstacles && (*obst)->isDynamic()) // we handle dynamic obstacles differently below
      continue;

    int index;

    if (cfg_->obstacles.obstacle_poses_affected >= teb_.sizePoses())
      index =  teb_.sizePoses() / 2;
    else
      index = teb_.findClosestTrajectoryPose(*(obst->get()));


    // check if obstacle is outside index-range between start and goal
    if ( (index <= 1) || (index > teb_.sizePoses()-2) ) // start and goal are fixed and findNearestBandpoint finds first or last conf if intersection point is outside the range
	    continue;

    if (inflated)
    {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }

    for (int neighbourIdx=0; neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected/2); neighbourIdx++)
    {
      if (index+neighbourIdx < teb_.sizePoses())
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information_inflated);
                dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_r = new EdgeObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information);
                dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
      }
      if ( index - neighbourIdx >= 0) // needs to be casted to int to allow negative values
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information_inflated);
                dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_l = new EdgeObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information);
                dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
      }
    }

  }
}

void TebOptimalPlanner::AddEdgesObstaclesForHumans() {
  if (cfg_->optim.weight_obstacle == 0 || obstacles_ == NULL)
    return;

  for (ObstContainer::const_iterator obst = obstacles_->begin();
       obst != obstacles_->end(); ++obst) {
    if ((*obst)->isDynamic()) // we handle dynamic obstacles differently below
      continue;

    unsigned int index;

    for (auto &human_teb_kv : humans_tebs_map_) {
      auto &human_teb = human_teb_kv.second;

      if (cfg_->obstacles.obstacle_poses_affected >= (int)human_teb.sizePoses())
        index = human_teb.sizePoses() / 2;
      else
        index = human_teb.findClosestTrajectoryPose(*(obst->get()));

      if ((index <= 1) || (index > human_teb.sizePoses() - 1))
        continue;

      Eigen::Matrix<double, 1, 1> information;
      information.fill(cfg_->optim.weight_obstacle);

      EdgeObstacle *dist_bandpt_obst = new EdgeObstacle;
      dist_bandpt_obst->setVertex(0, human_teb.PoseVertex(index));
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(
          *cfg_, static_cast<CircularRobotFootprintPtr>(human_model_).get(),
          obst->get());
      optimizer_->addEdge(dist_bandpt_obst);

      for (unsigned int neighbourIdx = 0;
           neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected / 2);
           neighbourIdx++) {
        if (index + neighbourIdx < human_teb.sizePoses()) {
          EdgeObstacle *dist_bandpt_obst_n_r = new EdgeObstacle;
          dist_bandpt_obst_n_r->setVertex(
              0, human_teb.PoseVertex(index + neighbourIdx));
          dist_bandpt_obst_n_r->setInformation(information);
          dist_bandpt_obst_n_r->setParameters(
              *cfg_, static_cast<CircularRobotFootprintPtr>(human_model_).get(),
              obst->get());
          optimizer_->addEdge(dist_bandpt_obst_n_r);
        }
        if ((int)index - (int)neighbourIdx >=
            0) { // TODO: may be > is enough instead of >=
          EdgeObstacle *dist_bandpt_obst_n_l = new EdgeObstacle;
          dist_bandpt_obst_n_l->setVertex(
              0, human_teb.PoseVertex(index - neighbourIdx));
          dist_bandpt_obst_n_l->setInformation(information);
          dist_bandpt_obst_n_l->setParameters(
              *cfg_, static_cast<CircularRobotFootprintPtr>(human_model_).get(),
              obst->get());
          optimizer_->addEdge(dist_bandpt_obst_n_l);
        }
      }
    }
  }
}

void TebOptimalPlanner::AddEdgesDynamicObstacles(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==NULL )
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1,1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0,1) = information(1,0) = 0;

  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (!(*obst)->isDynamic())
      continue;

    // Skip first and last pose, as they are fixed
    double time = teb_.TimeDiff(0);
    for (int i=1; i < teb_.sizePoses() - 1; ++i)
    {
      EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(time);
      dynobst_edge->setVertex(0,teb_.PoseVertex(i));
      dynobst_edge->setInformation(information);
      dynobst_edge->setParameters(*cfg_, robot_model_.get(), obst->get());
      optimizer_->addEdge(dynobst_edge);
      time += teb_.TimeDiff(i); // we do not need to check the time diff bounds, since we iterate to "< sizePoses()-1".
    }
  }
}

void TebOptimalPlanner::AddEdgesDynamicObstaclesForHumans(double weight_multiplier) {
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==NULL )
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1,1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0,1) = information(1,0) = 0;

  for (ObstContainer::const_iterator obst = obstacles_->begin();
       obst != obstacles_->end(); ++obst) {
    if (!(*obst)->isDynamic())
      continue;

    for (auto &human_teb_kv : humans_tebs_map_) {
      auto &human_teb = human_teb_kv.second;

      for (std::size_t i = 1; i < human_teb.sizePoses() - 1; ++i) {
        EdgeDynamicObstacle *dynobst_edge = new EdgeDynamicObstacle(i);
        dynobst_edge->setVertex(0, human_teb.PoseVertex(i));
        dynobst_edge->setVertex(1, human_teb.TimeDiffVertex(i));
        dynobst_edge->setInformation(information);
        dynobst_edge->setMeasurement(obst->get());
        dynobst_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(dynobst_edge);
      }
    }
  }
}

void TebOptimalPlanner::AddEdgesViaPoints()
{
  if (cfg_->optim.weight_viapoint==0 || via_points_==NULL || via_points_->empty() )
    return; // if weight equals zero skip adding edges!

  int start_pose_idx = 0;

  int n = teb_.sizePoses();
  if (n<3) // we do not have any degrees of freedom for reaching via-points
    return;

  for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it)
  {

    int index = teb_.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
    if (cfg_->trajectory.via_points_ordered)
      start_pose_idx = index+2; // skip a point to have a DOF inbetween for further via-points

    // check if point conicides with goal or is located behind it
    if ( index > n-2 )
      index = n-2; // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    if ( index < 1)
    {
      if (cfg_->trajectory.via_points_ordered)
      {
        index = 1; // try to connect the via point with the second (and non-fixed) pose. It is likely that autoresize adds new poses inbetween later.
      }
      else
      {
        ROS_DEBUG("TebOptimalPlanner::AddEdgesViaPoints(): skipping a via-point that is close or behind the current robot pose.");
        continue; // skip via points really close or behind the current robot pose
      }
    }
    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_->optim.weight_viapoint);

    EdgeViaPoint* edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0,teb_.PoseVertex(index));
    edge_viapoint->setInformation(information);
    edge_viapoint->setParameters(*cfg_, &(*vp_it));
    optimizer_->addEdge(edge_viapoint);
  }
}

void TebOptimalPlanner::AddEdgesViaPointsForHumans() {
  if (cfg_->optim.weight_human_viapoint == 0 || via_points_ == NULL ||
      via_points_->empty())
    return;

  int start_pose_idx = 0;

  int n = (int)teb_.sizePoses();
  if (n < 3)
    return;

  for (auto &human_via_points_kv : *humans_via_points_map_) {
    if (humans_tebs_map_.find(human_via_points_kv.first) == humans_tebs_map_.end()) {
      ROS_WARN_THROTTLE(THROTTLE_RATE, "inconsistant data between humans_tebs_map and humans_via_points_map (for id %ld)", human_via_points_kv.first);
      continue;
    }

    auto &human_via_points = human_via_points_kv.second;
    auto &human_teb = humans_tebs_map_[human_via_points_kv.first];

    for (ViaPointContainer::const_iterator vp_it = human_via_points.begin();
         vp_it != human_via_points.end(); ++vp_it) {
      int index =
          human_teb.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
      if (cfg_->trajectory.via_points_ordered)
        start_pose_idx = index + 2;

      if (index > n - 1)
        index = n - 1;
      if (index < 1)
        index = 1;

      Eigen::Matrix<double, 1, 1> information;
      information.fill(cfg_->optim.weight_human_viapoint);

      EdgeViaPoint *edge_viapoint = new EdgeViaPoint;
      edge_viapoint->setVertex(0, human_teb.PoseVertex(index));
      edge_viapoint->setInformation(information);
      edge_viapoint->setParameters(*cfg_, &(*vp_it));
      optimizer_->addEdge(edge_viapoint);
    }
  }
}

void TebOptimalPlanner::AddEdgesVelocity()
{
  if (cfg_->robot.max_vel_y == 0) // non-holonomic robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    int j = 0;
    Eigen::Matrix<double,2,2> information;
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_theta;
    information(0,1) = 0.0;
    information(1,0) = 0.0;

    for (auto &human_teb_kv : humans_tebs_map_) {
      auto &human_teb = human_teb_kv.second;
      for (int i=0; i < n - 1; ++i)
      {
        if(i<human_teb.sizePoses())
          j=i;

        EdgeVelocity* velocity_edge = new EdgeVelocity;
        velocity_edge->setVertex(0,teb_.PoseVertex(i));
        velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
        velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
        velocity_edge->setVertex(3, human_teb.PoseVertex(j));
        velocity_edge->setInformation(information);
        if(i<human_teb.sizePoses())
          velocity_edge->setParameters(*cfg_, robot_model_.get(), human_radius_);
        else
          velocity_edge->setParameters(*cfg_, robot_model_.get(), 0.0);
        optimizer_->addEdge(velocity_edge);
      }
    }
  }
  else // holonomic-robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_y==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    int j = 0;
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_y;
    information(2,2) = cfg_->optim.weight_max_vel_theta;

    for (auto &human_teb_kv : humans_tebs_map_) {
      auto &human_teb = human_teb_kv.second;
      for (int i=0; i < n - 1; ++i)
      {
        if(i<human_teb.sizePoses())
          j=i;
        EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
        velocity_edge->setVertex(0,teb_.PoseVertex(i));
        velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
        velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
        velocity_edge->setVertex(3, human_teb.PoseVertex(j));
        velocity_edge->setInformation(information);
        if(i<human_teb.sizePoses())
          velocity_edge->setParameters(*cfg_, robot_model_.get(), human_radius_);
        else
          velocity_edge->setParameters(*cfg_, robot_model_.get(), 0.0);
        optimizer_->addEdge(velocity_edge);
      }
    }
  }
}

void TebOptimalPlanner::AddEdgesVelocityForHumans() {
  if (cfg_->human.max_vel_y == 0) // non-holonomic robot
  {
    if ( cfg_->optim.weight_max_human_vel_x==0 && cfg_->optim.weight_max_human_vel_theta==0 && cfg_->optim.weight_nominal_human_vel_x == 0)
      return; // if weight equals zero skip adding edges!

    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_max_human_vel_x;
    information(1, 1) = cfg_->optim.weight_max_human_vel_theta;
    information(2, 2) = cfg_->optim.weight_nominal_human_vel_x;

    int itr_idx = 0 ;
    for (auto &human_teb_kv : humans_tebs_map_) {
      // human_nominal_vels[itr_idx]
      auto &human_teb = human_teb_kv.second;

      int n = human_teb.sizePoses();
      for (int i=0; i < n - 1; ++i)
      {
        EdgeVelocityHuman *human_velocity_edge = new EdgeVelocityHuman;
        human_velocity_edge->setVertex(0, human_teb.PoseVertex(i));
        human_velocity_edge->setVertex(1, human_teb.PoseVertex(i + 1));
        human_velocity_edge->setVertex(2, human_teb.TimeDiffVertex(i));
        human_velocity_edge->setInformation(information);
        human_velocity_edge->setParameters(*cfg_, human_nominal_vels[itr_idx]);
        optimizer_->addEdge(human_velocity_edge);
      }
      itr_idx++;
    }
  }
  else // holonomic-human
  {
    if ( cfg_->optim.weight_max_human_vel_x==0 && cfg_->optim.weight_max_human_vel_y==0 && cfg_->optim.weight_max_human_vel_theta==0 && cfg_->optim.weight_nominal_human_vel_x == 0)
      return; // if weight equals zero skip adding edges!

    // int n = human_teb.sizePoses();
    Eigen::Matrix<double,4,4> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_max_human_vel_x;
    information(1,1) = cfg_->optim.weight_max_human_vel_y;
    information(2,2) = cfg_->optim.weight_max_human_vel_theta;
    information(3,3) = cfg_->optim.weight_nominal_human_vel_x;

    int itr_idx = 0 ;
    for (auto &human_teb_kv : humans_tebs_map_) {
      auto &human_teb = human_teb_kv.second;

      int n = human_teb.sizePoses();
      for (int i=0; i < n - 1; ++i)
      {
        EdgeVelocityHolonomicHuman* human_velocity_edge = new EdgeVelocityHolonomicHuman;
        human_velocity_edge->setVertex(0,human_teb.PoseVertex(i));
        human_velocity_edge->setVertex(1,human_teb.PoseVertex(i+1));
        human_velocity_edge->setVertex(2,human_teb.TimeDiffVertex(i));
        human_velocity_edge->setInformation(information);
        human_velocity_edge->setParameters(*cfg_, human_nominal_vels[itr_idx]);
        // human_velocity_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(human_velocity_edge);
      }
      itr_idx++;
    }
  }
}

void TebOptimalPlanner::AddEdgesAcceleration()
{
  if (cfg_->optim.weight_acc_lim_x==0  && cfg_->optim.weight_acc_lim_theta==0)
    return; // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();

  if (cfg_->robot.max_vel_y == 0 || cfg_->robot.acc_lim_y == 0) // non-holonomic robot
  {
    Eigen::Matrix<double,2,2> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  }
  else // holonomic robot
  {
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_y;
    information(2,2) = cfg_->optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationHolonomicStart* acceleration_edge = new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAccelerationHolonomic* acceleration_edge = new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  }
}

void TebOptimalPlanner::AddEdgesAccelerationForHumans() {
  if (cfg_->optim.weight_human_acc_lim_x==0  && cfg_->optim.weight_human_acc_lim_theta==0)
    return; // if weight equals zero skip adding edges!

  if (cfg_->human.max_vel_y == 0 || cfg_->human.acc_lim_y == 0) // non-holonomic human
  {
    Eigen::Matrix<double,2,2> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_human_acc_lim_x;
    information(1,1) = cfg_->optim.weight_human_acc_lim_theta;
    for (auto &human_teb_kv : humans_tebs_map_)
    {
      auto &human_it = human_teb_kv.first;
      auto &human_teb = human_teb_kv.second;
      // check if an initial velocity should be taken into accound
      int n = human_teb.sizePoses();

      if (humans_vel_start_[human_it].first)
      {
        EdgeAccelerationHumanStart* human_acceleration_edge = new EdgeAccelerationHumanStart;
        human_acceleration_edge->setVertex(0,human_teb.PoseVertex(0));
        human_acceleration_edge->setVertex(1,human_teb.PoseVertex(1));
        human_acceleration_edge->setVertex(2,human_teb.TimeDiffVertex(0));
        human_acceleration_edge->setInitialVelocity(humans_vel_start_[human_it].second);
        human_acceleration_edge->setInformation(information);
        human_acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(human_acceleration_edge);
      }

      // now add the usual acceleration edge for each tuple of three teb poses
      for (int i=0; i < n - 2; ++i)
      {
        EdgeAccelerationHuman* human_acceleration_edge = new EdgeAccelerationHuman;
        human_acceleration_edge->setVertex(0,human_teb.PoseVertex(i));
        human_acceleration_edge->setVertex(1,human_teb.PoseVertex(i+1));
        human_acceleration_edge->setVertex(2,human_teb.PoseVertex(i+2));
        human_acceleration_edge->setVertex(3,human_teb.TimeDiffVertex(i));
        human_acceleration_edge->setVertex(4,human_teb.TimeDiffVertex(i+1));
        human_acceleration_edge->setInformation(information);
        human_acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(human_acceleration_edge);
      }

      // check if a goal velocity should be taken into accound
      if (humans_vel_goal_[human_it].first)
      {
        EdgeAccelerationHumanGoal* human_acceleration_edge = new EdgeAccelerationHumanGoal;
        human_acceleration_edge->setVertex(0,human_teb.PoseVertex(n-2));
        human_acceleration_edge->setVertex(1,human_teb.PoseVertex(n-1));
        human_acceleration_edge->setVertex(2,human_teb.TimeDiffVertex(human_teb.sizeTimeDiffs()-1 ));
        human_acceleration_edge->setGoalVelocity(humans_vel_goal_[human_it].second);
        human_acceleration_edge->setInformation(information);
        human_acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(human_acceleration_edge);
      }
    }
  }
  else // holonomic robot
  {
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_y;
    information(2,2) = cfg_->optim.weight_acc_lim_theta;
    for (auto &human_teb_kv : humans_tebs_map_)
    {
      auto &human_it = human_teb_kv.first;
      auto &human_teb = human_teb_kv.second;
      // check if an initial velocity should be taken into accound
      int n = human_teb.sizePoses();
      // check if an initial velocity should be taken into accound
      if (vel_start_.first)
      {
        EdgeAccelerationHolonomicHumanStart* human_acceleration_edge = new EdgeAccelerationHolonomicHumanStart;
        human_acceleration_edge->setVertex(0,human_teb.PoseVertex(0));
        human_acceleration_edge->setVertex(1,human_teb.PoseVertex(1));
        human_acceleration_edge->setVertex(2,human_teb.TimeDiffVertex(0));
        human_acceleration_edge->setInitialVelocity(humans_vel_start_[human_it].second);
        human_acceleration_edge->setInformation(information);
        human_acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(human_acceleration_edge);
      }

      // now add the usual acceleration edge for each tuple of three teb poses
      for (int i=0; i < n - 2; ++i)
      {
        EdgeAccelerationHolonomicHuman* human_acceleration_edge = new EdgeAccelerationHolonomicHuman;
        human_acceleration_edge->setVertex(0,human_teb.PoseVertex(i));
        human_acceleration_edge->setVertex(1,human_teb.PoseVertex(i+1));
        human_acceleration_edge->setVertex(2,human_teb.PoseVertex(i+2));
        human_acceleration_edge->setVertex(3,human_teb.TimeDiffVertex(i));
        human_acceleration_edge->setVertex(4,human_teb.TimeDiffVertex(i+1));
        human_acceleration_edge->setInformation(information);
        human_acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(human_acceleration_edge);
      }

      // check if a goal velocity should be taken into accound
      if (vel_goal_.first)
      {
        EdgeAccelerationHolonomicHumanGoal* human_acceleration_edge = new EdgeAccelerationHolonomicHumanGoal;
        human_acceleration_edge->setVertex(0,human_teb.PoseVertex(n-2));
        human_acceleration_edge->setVertex(1,human_teb.PoseVertex(n-1));
        human_acceleration_edge->setVertex(2,human_teb.TimeDiffVertex(human_teb.sizeTimeDiffs()-1 ));
        human_acceleration_edge->setGoalVelocity(humans_vel_goal_[human_it].second);
        human_acceleration_edge->setInformation(information);
        human_acceleration_edge->setHATebConfig(*cfg_);
        optimizer_->addEdge(human_acceleration_edge);
      }
    }
  }
}


void TebOptimalPlanner::AddEdgesTimeOptimal()
{
  if (cfg_->optim.weight_optimaltime==0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_optimaltime);

  for (int i=0; i < teb_.sizeTimeDiffs(); ++i)
  {
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0,teb_.TimeDiffVertex(i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setHATebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}

void TebOptimalPlanner::AddEdgesTimeOptimalForHumans() {
  if (cfg_->optim.weight_human_optimaltime == 0) {
    return;
  }

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_human_optimaltime);

  for (auto &human_teb_kv : humans_tebs_map_) {
    auto &human_teb = human_teb_kv.second;

    std::size_t NoTimeDiffs(human_teb.sizeTimeDiffs());
    for (std::size_t i = 0; i < NoTimeDiffs; ++i) {
      EdgeTimeOptimal *timeoptimal_edge = new EdgeTimeOptimal;
      timeoptimal_edge->setVertex(0, human_teb.TimeDiffVertex(i));
      timeoptimal_edge->setInformation(information);
      timeoptimal_edge->setHATebConfig(*cfg_);
      // timeoptimal_edge->setInitialTime(human_teb.TimeDiffVertex(i)->dt());
      optimizer_->addEdge(timeoptimal_edge);
    }
  }
}

void TebOptimalPlanner::AddEdgesShortestPath()
{
  if (cfg_->optim.weight_shortest_path==0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_shortest_path);

  for (int i=0; i < teb_.sizePoses()-1; ++i)
  {
    EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0,teb_.PoseVertex(i));
    shortest_path_edge->setVertex(1,teb_.PoseVertex(i+1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setHATebConfig(*cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}


void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;

  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setHATebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}

void TebOptimalPlanner::AddEdgesKinematicsDiffDriveForHumans() {
  if (cfg_->optim.weight_kinematics_nh == 0 &&
      cfg_->optim.weight_kinematics_forward_drive == 0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 2, 2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;

  for (auto &human_teb_kv : humans_tebs_map_) {
    auto &human_teb = human_teb_kv.second;
    for (unsigned int i = 0; i < human_teb.sizePoses() - 1; i++) {
      EdgeKinematicsDiffDrive *kinematics_edge = new EdgeKinematicsDiffDrive;
      kinematics_edge->setVertex(0, human_teb.PoseVertex(i));
      kinematics_edge->setVertex(1, human_teb.PoseVertex(i + 1));
      kinematics_edge->setInformation(information_kinematics);
      kinematics_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(kinematics_edge);
    }
  }
}

void TebOptimalPlanner::AddEdgesKinematicsCarlike()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_turning_radius==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;

  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setHATebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}

void TebOptimalPlanner::AddEdgesKinematicsCarlikeForHumans()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_turning_radius==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;

  for (auto &human_teb_kv : humans_tebs_map_) {
    auto &human_teb = human_teb_kv.second;
    for (int i=0; i < human_teb.sizePoses()-1; i++) // ignore twiced start only
    {
      EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
      kinematics_edge->setVertex(0,human_teb.PoseVertex(i));
      kinematics_edge->setVertex(1,human_teb.PoseVertex(i+1));
      kinematics_edge->setInformation(information_kinematics);
      kinematics_edge->setHATebConfig(*cfg_);
      optimizer_->addEdge(kinematics_edge);
    }
  }
}

void TebOptimalPlanner::AddEdgesPreferRotDir()
{
  //TODO(roesmann): Note, these edges can result in odd predictions, in particular
  //                we can observe a substantional mismatch between open- and closed-loop planning
  //                leading to a poor control performance.
  //                At the moment, we keep these functionality for oscillation recovery:
  //                Activating the edge for a short time period might not be crucial and
  //                could move the robot to a new oscillation-free state.
  //                This needs to be analyzed in more detail!
  if (prefer_rotdir_ == RotType::none || cfg_->optim.weight_prefer_rotdir==0)
    return; // if weight equals zero skip adding edges!

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left)
  {
    ROS_WARN("TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType selected. Skipping edge creation.");
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,1,1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);

  for (int i=0; i < teb_.sizePoses()-1 && i < 3; ++i) // currently: apply to first 3 rotations
  {
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0,teb_.PoseVertex(i));
    rotdir_edge->setVertex(1,teb_.PoseVertex(i+1));
    rotdir_edge->setInformation(information_rotdir);

    if (prefer_rotdir_ == RotType::left)
        rotdir_edge->preferLeft();
    else if (prefer_rotdir_ == RotType::right)
        rotdir_edge->preferRight();

    optimizer_->addEdge(rotdir_edge);
  }
}

void TebOptimalPlanner::AddEdgesHumanRobotSafety() {
  auto robot_teb_size = (int)teb_.sizePoses();

  double min_dist_ = cfg_->hateb.min_human_robot_dist;
  double weight_safety = cfg_->optim.weight_human_robot_safety;
  // if(isMode==1){
  //   weight_safety = 1.0;
  // }

  if(current_human_robot_min_dist < 2.0){
    for (auto &human_teb_kv : humans_tebs_map_) {
      auto &human_teb = human_teb_kv.second;

      for (unsigned int i = 0;
           (i < human_teb.sizePoses()) && (i < robot_teb_size); i++) {
        Eigen::Matrix<double, 1, 1> information_human_robot;
        information_human_robot.fill(weight_safety);
        EdgeHumanRobotSafety *human_robot_safety_edge = new EdgeHumanRobotSafety;
        human_robot_safety_edge->setVertex(0, teb_.PoseVertex(i));
        human_robot_safety_edge->setVertex(1, human_teb.PoseVertex(i));
        human_robot_safety_edge->setInformation(information_human_robot);
        human_robot_safety_edge->setParameters(*cfg_, robot_model_.get(),
                                               human_radius_, min_dist_);
        optimizer_->addEdge(human_robot_safety_edge);
      }
    }
  }
}

void TebOptimalPlanner::AddEdgesHumanHumanSafety() {
  for (auto oi = humans_tebs_map_.begin(); oi != humans_tebs_map_.end();) {
    auto &human1_teb = oi->second;
    for (auto ii = ++oi; ii != humans_tebs_map_.end(); ii++) {
      auto &human2_teb = ii->second;

      for (unsigned int k = 0;
           (k < human1_teb.sizePoses()) && (k < human2_teb.sizePoses()); k++) {
        Eigen::Matrix<double, 1, 1> information_human_human;
        information_human_human.fill(cfg_->optim.weight_human_human_safety);

        EdgeHumanHumanSafety *human_human_safety_edge =
            new EdgeHumanHumanSafety;
        human_human_safety_edge->setVertex(0, human1_teb.PoseVertex(k));
        human_human_safety_edge->setVertex(1, human2_teb.PoseVertex(k));
        human_human_safety_edge->setInformation(information_human_human);
        human_human_safety_edge->setParameters(*cfg_, human_radius_);
        optimizer_->addEdge(human_human_safety_edge);
      }
    }
  }
}

void TebOptimalPlanner::AddEdgesHumanRobotTTC() {
  Eigen::Matrix<double, 1, 1> information_human_robot_ttc;
  information_human_robot_ttc.fill(cfg_->optim.weight_human_robot_ttc);

  auto robot_teb_size = (int)teb_.sizePoses();

  for (auto &human_teb_kv : humans_tebs_map_) {
    auto &human_teb = human_teb_kv.second;

    size_t human_teb_size = human_teb.sizePoses();
    for (unsigned int i = 0;
         (i < human_teb_size - 1) && (i < robot_teb_size - 1); i++) {

      EdgeHumanRobotTTC *human_robot_ttc_edge = new EdgeHumanRobotTTC;
      human_robot_ttc_edge->setVertex(0, teb_.PoseVertex(i));
      human_robot_ttc_edge->setVertex(1, teb_.PoseVertex(i + 1));
      human_robot_ttc_edge->setVertex(2, teb_.TimeDiffVertex(i));
      human_robot_ttc_edge->setVertex(3, human_teb.PoseVertex(i));
      human_robot_ttc_edge->setVertex(4, human_teb.PoseVertex(i + 1));
      human_robot_ttc_edge->setVertex(5, human_teb.TimeDiffVertex(i));
      human_robot_ttc_edge->setInformation(information_human_robot_ttc);
      human_robot_ttc_edge->setParameters(*cfg_, robot_radius_, human_radius_);
      optimizer_->addEdge(human_robot_ttc_edge);
    }
  }
}


void TebOptimalPlanner::AddEdgesHumanRobotTTCplus() {
  Eigen::Matrix<double, 1, 1> information_human_robot_ttcplus;
  information_human_robot_ttcplus.fill(cfg_->optim.weight_human_robot_ttcplus);


   auto robot_teb_size = teb_.sizePoses();
  for (auto &human_teb_kv : humans_tebs_map_) {
    auto &human_teb = human_teb_kv.second;
     size_t human_teb_size = human_teb.sizePoses();
    for (unsigned int i = 0;
         (i < human_teb_size - 1) && (i < robot_teb_size - 1) && (isMode==0); i++) {

      EdgeHumanRobotTTCplus *human_robot_ttcplus_edge = new EdgeHumanRobotTTCplus();
      human_robot_ttcplus_edge->setVertex(0, teb_.PoseVertex(i));
      human_robot_ttcplus_edge->setVertex(1, teb_.PoseVertex(i + 1));
      human_robot_ttcplus_edge->setVertex(2, teb_.TimeDiffVertex(i));
      human_robot_ttcplus_edge->setVertex(3, human_teb.PoseVertex(i));
      human_robot_ttcplus_edge->setVertex(4, human_teb.PoseVertex(i + 1));
      human_robot_ttcplus_edge->setVertex(5, human_teb.TimeDiffVertex(i));
      human_robot_ttcplus_edge->setInformation(information_human_robot_ttcplus);
      human_robot_ttcplus_edge->setParameters(*cfg_, robot_radius_, human_radius_);
      optimizer_->addEdge(human_robot_ttcplus_edge);
    }
  }
}

void TebOptimalPlanner::AddEdgesHumanRobotRelVelocity() {
  Eigen::Matrix<double, 1, 1> information_human_robot_rel_vel;
  information_human_robot_rel_vel.fill(cfg_->optim.weight_human_robot_rel_vel);

  auto robot_teb_size = (int)teb_.sizePoses();
  for (auto &human_teb_kv : humans_tebs_map_) {
    auto &human_teb = human_teb_kv.second;

    size_t human_teb_size = human_teb.sizePoses();
    for (unsigned int i = 0; (i < human_teb_size - 1) && (i < robot_teb_size - 1); i++) {

      EdgeHumanRobotRelVelocity *human_robot_rel_vel_edge = new EdgeHumanRobotRelVelocity;
      human_robot_rel_vel_edge->setVertex(0, teb_.PoseVertex(i));
      human_robot_rel_vel_edge->setVertex(1, teb_.PoseVertex(i + 1));
      human_robot_rel_vel_edge->setVertex(2, teb_.TimeDiffVertex(i));
      human_robot_rel_vel_edge->setVertex(3, human_teb.PoseVertex(i));
      human_robot_rel_vel_edge->setVertex(4, human_teb.PoseVertex(i + 1));
      human_robot_rel_vel_edge->setVertex(5, human_teb.TimeDiffVertex(i));
      human_robot_rel_vel_edge->setInformation(information_human_robot_rel_vel);
      human_robot_rel_vel_edge->setParameters(*cfg_);
      optimizer_->addEdge(human_robot_rel_vel_edge);
    }
  }
}

void TebOptimalPlanner::AddEdgesHumanRobotVisibility() {
    auto robot_teb_size = (int)teb_.sizePoses();

    for (auto &human_teb_kv : humans_tebs_map_) {
        auto &human_teb = human_teb_kv.second;

        for (unsigned int i = 0;
             (i < human_teb.sizePoses()) && (i < robot_teb_size); i++) {
            Eigen::Matrix<double, 1, 1> information_human_robot;
            information_human_robot.fill(cfg_->optim.weight_human_robot_visibility);

            EdgeHumanRobotVisibility *human_robot_visibility_edge = new EdgeHumanRobotVisibility;
            human_robot_visibility_edge->setVertex(0, teb_.PoseVertex(i));
            human_robot_visibility_edge->setVertex(1, human_teb.PoseVertex(i));
            human_robot_visibility_edge->setInformation(information_human_robot);
            human_robot_visibility_edge->setParameters(*cfg_);
            optimizer_->addEdge(human_robot_visibility_edge);
        }
    }
}

void TebOptimalPlanner::AddVertexEdgesApproach() {
  if (!approach_pose_vertex) {
    ROS_ERROR("approch pose vertex does not exist");
    return;
  }
  double min_dist_ = cfg_->hateb.min_human_robot_dist;
  if(isMode==1){
    min_dist_ = 0.2;
  }

  Eigen::Matrix<double, 1, 1> information_approach;
  information_approach.fill(cfg_->optim.weight_obstacle);

  for (auto &teb_pose : teb_.poses()) {
    EdgeHumanRobotSafety *approach_edge = new EdgeHumanRobotSafety;
    approach_edge->setVertex(0, teb_pose);
    approach_edge->setVertex(1, approach_pose_vertex);
    approach_edge->setInformation(information_approach);
    approach_edge->setParameters(*cfg_, robot_model_.get(), human_radius_, min_dist_);
    optimizer_->addEdge(approach_edge);
  }
}

void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost, hateb_local_planner::OptimizationCostArray *op_costs)
{
  // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty())
  {
    // here the graph is build again, for time efficiency make sure to call this function
    // between buildGraph and Optimize (deleted), but it depends on the application
    buildGraph();
    optimizer_->initializeOptimization();
  }
  else
  {
    graph_exist_flag = true;
  }

  optimizer_->computeInitialGuess();

  cost_ = 0;
  double time_opt_cost = 0.0, kinematics_dd_cost = 0.0, shortest_path_cost =0.0,
         kinematics_cl_cost = 0.0, robot_vel_cost = 0.0, human_vel_cost = 0.0, robot_vel_holo_cost=0.0,robot_acc_holo_cost=0.0,
         robot_acc_cost = 0.0, human_acc_cost = 0.0, human_vel_holo_cost =0.0,human_acc_holo_cost=0.0, obst_cost = 0.0,rot_dir_cost=0.0,
         dyn_obst_cost = 0.0, via_cost = 0.0, hr_safety_cost = 0.0,
         hh_safety_cost = 0.0, hr_ttc_cost = 0.0, hr_ttclosest_cost = 0.0 ,hr_ttcplus_cost = 0.0 ,  hr_rel_vel_cost = 0.0, hr_visi_cost = 0.0,ttcplus_error=0.0;

  std::vector<double> time_opt_cost_vector, kinematics_dd_cost_vector, kinematics_cl_cost_vector,
                      robot_vel_cost_vector, human_vel_cost_vector, robot_vel_holo_cost_vector,human_vel_holo_cost_vector, robot_acc_cost_vector,robot_acc_holo_cost_vector,
                      human_acc_cost_vector, human_acc_holo_cost_vector, obst_cost_vector, dyn_obst_cost_vector, via_cost_vector, hr_safety_cost_vector,
                      hh_safety_cost_vector, hr_ttc_cost_vector, hr_ttclosest_cost_vector, hr_ttcplus_cost_vector,
                      hr_rel_vel_cost_vector, hr_visi_cost_vector, shortest_path_cost_vector;

  bool f1=false,f2=false,f3=false,f4=false,f5=false;
  double ttc_first, ttcplus_first, obs_first, safety_first, visible_first;

  if (alternative_time_cost)
  {
    cost_ += teb_.getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for similar TEBs,
    // since we are using an AutoResize Function with hysteresis.
  }

  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  int i=0;
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it = optimizer_->activeEdges().begin(); it!= optimizer_->activeEdges().end(); it++)
  { i++;
    double cur_cost = (*it)->chi2();

    // if (dynamic_cast<EdgeObstacle*>(*it) != nullptr
    //     || dynamic_cast<EdgeInflatedObstacle*>(*it) != nullptr
    //     || dynamic_cast<EdgeDynamicObstacle*>(*it) != nullptr)
    // {
    //   cur_cost *= obst_cost_scale;
    // }
    // else if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr)
    // {
    //   cur_cost *= viapoint_cost_scale;
    // }
    // else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr && alternative_time_cost)
    // {
    //   continue; // skip these edges if alternative_time_cost is active
    // }


    ////////////////////////////////////////////////////////////////////////////////////////

    if (dynamic_cast<EdgeTimeOptimal *>(*it) != nullptr && !alternative_time_cost) {
      time_opt_cost += cur_cost;
      time_opt_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout<< "EdgeTimeOptimal " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeShortestPath *>(*it) != nullptr) {
      shortest_path_cost += cur_cost;
      shortest_path_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout << "EdgeShortestPath " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgePreferRotDir *>(*it) != nullptr) {
      rot_dir_cost += cur_cost;
      cost_ += cur_cost;
      // std::cout << "EdgePreferRotDir " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeKinematicsDiffDrive *>(*it) != nullptr) {
      kinematics_dd_cost += cur_cost;
      kinematics_dd_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout << "EdgeKinematicsDiffDrive " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeKinematicsCarlike *>(*it) != nullptr) {
      kinematics_cl_cost += cur_cost;
      kinematics_cl_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout << "EdgeKinematicsCarlike " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeVelocity *>(*it) != nullptr) {
      robot_vel_cost += cur_cost;
      robot_vel_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout << "EdgeVelocity " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeVelocityHolonomic *>(*it) != nullptr) {
      robot_vel_holo_cost += cur_cost;
      robot_vel_holo_cost_vector.push_back(cur_cost);

      cost_ += cur_cost;
      // std::cout << "EdgeVelocityHolonomic " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeVelocityHuman *>(*it) != nullptr) {
      human_vel_cost += cur_cost;
      human_vel_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout << "EdgeVelocityHuman " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeVelocityHolonomicHuman *>(*it) != nullptr) {
      human_vel_holo_cost += cur_cost;
      human_vel_holo_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout << "EdgeVelocityHolonomicHuman " << cur_cost<< '\n';
      continue;
    }

    // TODO: add cost of start and goal accelerations

    if (dynamic_cast<EdgeAcceleration *>(*it) != nullptr) {
      robot_acc_cost += cur_cost;
      robot_acc_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout << "EdgeAcceleration " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeAccelerationHolonomic *>(*it) != nullptr) {
      robot_acc_holo_cost += cur_cost;
      robot_acc_holo_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout << "EdgeAccelerationHolonomic " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeAccelerationHuman *>(*it) != nullptr) {
      human_acc_cost += cur_cost;
      human_acc_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout << "EdgeAccelerationHuman " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeAccelerationHolonomicHuman *>(*it) != nullptr) {
      human_acc_holo_cost += cur_cost;
      human_acc_holo_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout << "EdgeAccelerationHolonomicHuman " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeObstacle *>(*it) != nullptr) {
      cur_cost *= obst_cost_scale;
      obst_cost += cur_cost;
      cost_ += cur_cost;
      obst_cost_vector.push_back(cur_cost);
      if(!f5){
        obs_first = cur_cost;
        f5=true;
      }
      // std::cout << "EdgeObstacle " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeInflatedObstacle*>(*it) !=nullptr)
    {
      cur_cost *= obst_cost_scale;
      cost_ += cur_cost;
      // std::cout << "EdgeInflatedObstacle " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeDynamicObstacle *>(*it) != nullptr) {
      cur_cost *= obst_cost_scale;
      dyn_obst_cost += cur_cost;
      dyn_obst_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout << "EdgeDynamicObstacle " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeViaPoint *>(*it) != nullptr) {
      cur_cost *= viapoint_cost_scale;
      via_cost += cur_cost;
      via_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout << "EdgeViaPoint " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeHumanRobotSafety *>(*it) != nullptr) {
      hr_safety_cost += cur_cost;
      hr_safety_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      if(!f1){
        safety_first = cur_cost;
        f1=true;
      }
      // EdgeHumanRobotSafety *edge_human_robot_safety = dynamic_cast<EdgeHumanRobotSafety *>(*it);
      // const VertexPose *robot_bandpt = static_cast<const VertexPose *>(edge_human_robot_safety->vertex(0));
      // std::cout << "edge_human_robot_safety->vertices() "<<robot_bandpt->pose()<< '\n';
      // std::cout << "i " << i<< '\n';
      // std::cout << "teb_.size() " <<teb_.sizePoses()<< '\n';
      // std::cout << "teb_.Pose(0) "<<teb_.Pose(0) << '\n';
      // std::cout << "teb_.Pose(1) "<<teb_.Pose(1) << '\n';
      // std::cout << "teb_.Pose(2) "<<teb_.Pose(2) << '\n';
      // std::cout << "EdgeHumanRobotSafety " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeHumanHumanSafety *>(*it) != nullptr) {
      hh_safety_cost += cur_cost;
      hh_safety_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      // std::cout << "EdgeHumanHumanSafety " << cur_cost<< '\n';
      continue;
    }

    if (dynamic_cast<EdgeHumanRobotTTC *>(*it) != nullptr) {
      hr_ttc_cost += cur_cost;
      hr_ttc_cost_vector.push_back(cur_cost);

      cost_ += cur_cost;
      // std::cout << "EdgeHumanRobotTTC " << cur_cost<< '\n';
      if(!f2){
        ttc_first = cur_cost;
        f2=true;
      }
      continue;
    }

    // if (dynamic_cast<EdgeHumanRobotTTCplus *>(*it) != nullptr) {
    //   hr_ttcplus_cost += cur_cost;
    //   cost_ += cur_cost;
    //   // std::cout << "EdgeHumanRobotTTCplus " << cur_cost<< '\n';
    //   continue;
    // }

    EdgeHumanRobotTTCplus *edge_human_robot_ttcplus = dynamic_cast<EdgeHumanRobotTTCplus *>(*it);
    if (edge_human_robot_ttcplus != NULL) {
      cost_ += edge_human_robot_ttcplus->getError().squaredNorm();
      hr_ttcplus_cost += edge_human_robot_ttcplus->getError().squaredNorm();
      ttcplus_error += edge_human_robot_ttcplus->getError()[0];
      hr_ttcplus_cost_vector.push_back(edge_human_robot_ttcplus->getError().squaredNorm());
      // std::cout << "Entered here" << '\n';
      if(!f3){
        ttcplus_first = hr_ttcplus_cost;
        f3=true;
      }
      // std::cout << "i " << i<< '\n';
      // std::cout << "teb_.size() " <<teb_.sizePoses()<< '\n';
      // std::cout << "EdgeHumanRobotTTCplus " << hr_ttcplus_cost<< '\n';

      continue;
    }

    if (dynamic_cast<EdgeHumanRobotRelVelocity *>(*it) != nullptr) {
      hr_rel_vel_cost += cur_cost;
      hr_rel_vel_cost_vector.push_back(cur_cost);
      cost_ += cur_cost;
      continue;
    }

    if (dynamic_cast<EdgeHumanRobotVisibility *>(*it) != nullptr) {
        hr_visi_cost += cur_cost;
        hr_visi_cost_vector.push_back(cur_cost);
        cost_ += cur_cost;
        // std::cout << "EdgeHumanRobotVisibility " << cur_cost<< '\n';
        if(!f4){
          visible_first = cur_cost;
          f4=true;
        }
        continue;
    }

    // {std::cout << "i " << i<< '\n';
      // std::cout << "hr_ttcplus_cost: " << hr_ttcplus_cost <<'\n';
      // std::cout << "hr_safety_cost: " << hr_safety_cost <<'\n';

    // }
    // std::cout << "i " <<i << '\n';
  }

  if (op_costs) {
    // std::cout << "I entered inside the op_costs" << '\n';
    op_costs->costs.clear();

    // std::ofstream outdata;
    // outdata.open("/home/ptsingaman/ros_ws/Navigation_planner_melodic/dir_data.dat");
    // if( !outdata ){
    //   ROS_INFO("Cannot open the file to write");
    // }
    //
    // for(int indx=0;indx<hr_dir_cost_vector.size();indx++){
    //   outdata << hr_dir_cost_vector[indx] << std::endl;
    //   // std::cout << "dir_cost" << hr_dir_cost_vector[indx]<< '\n';
    // }
    // outdata.close();

    hateb_local_planner::OptimizationCost optc;

    optc.type = hateb_local_planner::OptimizationCost::TIME_OPTIMALITY;
    optc.cost = time_opt_cost;
    optc.costs_arr = time_opt_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::KINEMATIC_DD;
    optc.cost = kinematics_dd_cost;
    optc.costs_arr = kinematics_dd_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::KINEMATIC_CL;
    optc.cost = kinematics_cl_cost;
    optc.costs_arr = kinematics_cl_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::ROBOT_VEL;
    optc.cost = robot_vel_holo_cost;
    optc.costs_arr = robot_vel_holo_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::HUMAN_VEL;
    optc.cost = human_vel_holo_cost;
    optc.costs_arr = human_vel_holo_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::ROBOT_ACC;
    optc.cost = robot_acc_holo_cost;
    optc.costs_arr = robot_acc_holo_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::HUMAN_ACC;
    optc.cost = human_acc_holo_cost;
    optc.costs_arr = human_acc_holo_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::OBSTACLE;
    // optc.cost = obst_cost;
    optc.cost = obs_first;
    optc.costs_arr = obst_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::DYNAMIC_OBSTACLE;
    optc.cost = dyn_obst_cost;
    optc.costs_arr = dyn_obst_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::VIA_POINT;
    optc.cost = via_cost;
    optc.costs_arr = via_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::HUMAN_ROBOT_SAFETY;
    optc.cost = hr_safety_cost;
    // optc.cost = safety_first;
    optc.costs_arr = hr_safety_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::HUMAN_HUMAN_SAFETY;
    optc.cost = hh_safety_cost;
    optc.costs_arr = hh_safety_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::HUMAN_ROBOT_TTC;
    optc.cost = hr_ttc_cost;
    optc.costs_arr = hr_ttc_cost_vector;
    // optc.cost = ttc_first;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::HUMAN_ROBOT_TTClosest;                //michele
    optc.cost = hr_ttclosest_cost;
    optc.costs_arr = hr_ttclosest_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::HUMAN_ROBOT_TTCplus;                //michele
    optc.cost = hr_ttcplus_cost;
    optc.costs_arr = hr_ttcplus_cost_vector;
    // optc.cost = ttcplus_first;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::HUMAN_ROBOT_REL_VEL;
    optc.cost = hr_rel_vel_cost;
    optc.costs_arr = hr_rel_vel_cost_vector;
    op_costs->costs.push_back(optc);

    optc.type = hateb_local_planner::OptimizationCost::HUMAN_ROBOT_VISIBILITY;
    optc.cost = hr_visi_cost;
    optc.costs_arr = hr_visi_cost_vector;
    // optc.cost = visible_first;
    op_costs->costs.push_back(optc);
  }

  ROS_DEBUG("Costs:\n\ttime_opt_cost = %.2f\n\tkinematics_dd_cost = "
            "%.2f\n\tkinematics_cl_cost = %.2f\n\trobot_vel_cost = "
            "%.2f\n\thuman_vel_cost = %.2f\n\trobot_acc_cost = "
            "%.2f\n\thuman_acc_cost = %.2f\n\tobst_cost = "
            "%.2f\n\tdyn_obst_cost = %.2f\n\tvia_cost = "
            "%.2f\n\thr_safety_cost = %.2f\n\thh_safety_cost = "
            "%.2f\n\thr_ttc_cost =   %.2f\n\thr_rel_vel_cost = "
            "%.2f\n\thr_ttclosest_cost = %.2f\n\thr_ttcplus_cost = "            //michele
            "%.2f\n\thr_visi_cost = %.2f\n\ttotal_tab_time = %.2f",
            time_opt_cost, kinematics_dd_cost, kinematics_cl_cost,
            robot_vel_cost, human_vel_cost, robot_acc_cost, human_acc_cost,
            obst_cost, dyn_obst_cost, via_cost, hr_safety_cost, hh_safety_cost,
            hr_ttc_cost,hr_rel_vel_cost,hr_ttclosest_cost,hr_ttcplus_cost, hr_visi_cost, teb_.getSumOfAllTimeDiffs());

  // delete temporary created graph
  if (!graph_exist_flag)
    clearGraph();
}


void TebOptimalPlanner::extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const
{
  if (dt == 0)
  {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }

  Eigen::Vector2d deltaS = pose2.position() - pose1.position();

  if (cfg_->robot.max_vel_y == 0) // nonholonomic robot
  {
    Eigen::Vector2d conf1dir( cos(pose1.theta()), sin(pose1.theta()) );
    // translational velocity
    double dir = deltaS.dot(conf1dir);
    vx = (double) g2o::sign(dir) * deltaS.norm()/dt;
    vy = 0;
  }

  else // holonomic robot
  {
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
    double p1_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
    vx = p1_dx / dt;
    vy = p1_dy / dt;
  }

  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff/dt;
}

bool TebOptimalPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses, double dt_ref) const
{
  if (teb_.sizePoses()<2)
  {
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }

  look_ahead_poses = std::max(1, std::min(look_ahead_poses, (int)teb_.sizePoses() - 1));
  double dt = 0.0;
  for(int counter = 0; counter < look_ahead_poses; ++counter)
  {
    dt += teb_.TimeDiff(counter);
    if(dt >= dt_ref * look_ahead_poses)  // TODO: change to look-ahead time? Refine trajectory?
    {
        look_ahead_poses = counter + 1;
        break;
    }
  }

  if (dt<=0)
  {
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }

  // Get velocity from the first two configurations
extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);

  return true;
}

void TebOptimalPlanner::getVelocityProfile(std::vector<geometry_msgs::Twist>& velocity_profile) const
{
  int n = teb_.sizePoses();
  velocity_profile.resize( n+1 );

  // start velocity
  velocity_profile.front().linear.z = 0;
  velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;
  velocity_profile.front().linear.x = vel_start_.second.linear.x;
  velocity_profile.front().linear.y = vel_start_.second.linear.y;
  velocity_profile.front().angular.z = vel_start_.second.angular.z;

  for (int i=1; i<n; ++i)
  {
    velocity_profile[i].linear.z = 0;
    velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), velocity_profile[i].linear.x, velocity_profile[i].linear.y, velocity_profile[i].angular.z);
  }

  // goal velocity
  velocity_profile.back().linear.z = 0;
  velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;
  velocity_profile.back().linear.x = vel_goal_.second.linear.x;
  velocity_profile.back().linear.y = vel_goal_.second.linear.y;
  velocity_profile.back().angular.z = vel_goal_.second.angular.z;
}

void TebOptimalPlanner::getFullTrajectory(std::vector<TrajectoryPointMsg>& trajectory) const
{
  int n = teb_.sizePoses();

  trajectory.resize(n);

  if (n == 0)
    return;

  double curr_time = 0;

  // start
  TrajectoryPointMsg& start = trajectory.front();
  teb_.Pose(0).toPoseMsg(start.pose);
  start.velocity.linear.z = 0;
  start.velocity.angular.x = start.velocity.angular.y = 0;
  start.velocity.linear.x = vel_start_.second.linear.x;
  start.velocity.linear.y = vel_start_.second.linear.y;
  start.velocity.angular.z = vel_start_.second.angular.z;
  start.time_from_start.fromSec(curr_time);

  curr_time += teb_.TimeDiff(0);

  // intermediate points
  for (int i=1; i < n-1; ++i)
  {
    TrajectoryPointMsg& point = trajectory[i];
    teb_.Pose(i).toPoseMsg(point.pose);
    point.velocity.linear.z = 0;
    point.velocity.angular.x = point.velocity.angular.y = 0;
    double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), vel1_x, vel1_y, omega1);
    extractVelocity(teb_.Pose(i), teb_.Pose(i+1), teb_.TimeDiff(i), vel2_x, vel2_y, omega2);
    point.velocity.linear.x = 0.5*(vel1_x+vel2_x);
    point.velocity.linear.y = 0.5*(vel1_y+vel2_y);
    point.velocity.angular.z = 0.5*(omega1+omega2);
    point.time_from_start.fromSec(curr_time);

    curr_time += teb_.TimeDiff(i);
  }

  // goal
  TrajectoryPointMsg& goal = trajectory.back();
  teb_.BackPose().toPoseMsg(goal.pose);
  goal.velocity.linear.z = 0;
  goal.velocity.angular.x = goal.velocity.angular.y = 0;
  goal.velocity.linear.x = vel_goal_.second.linear.x;
  goal.velocity.linear.y = vel_goal_.second.linear.y;
  goal.velocity.angular.z = vel_goal_.second.angular.z;
  goal.time_from_start.fromSec(curr_time);
}

void TebOptimalPlanner::getFullHumanTrajectory(const uint64_t human_id, std::vector<TrajectoryPointMsg> &human_trajectory) {
  auto human_teb_it = humans_tebs_map_.find(human_id);
  if (human_teb_it != humans_tebs_map_.end()) {
    auto &human_teb = human_teb_it->second;
    auto human_teb_size = human_teb.sizePoses();
    if (human_teb_size < 3) {
      ROS_WARN("TEB size is %ld for human %ld", human_teb_size, human_id);
      return;
    }

    human_trajectory.resize(human_teb_size);

    double curr_time = 0;

    // start
    TrajectoryPointMsg &start = human_trajectory.front();
    // std::cout << "Human Teb Pose: 0\n" << start.pose.position << '\n';
    human_teb.Pose(0).toPoseMsg(start.pose);
    start.velocity.linear.z = 0;
    start.velocity.angular.x = start.velocity.angular.y = 0;
    start.velocity.linear.x = humans_vel_start_[human_id].second.linear.x;
    start.velocity.linear.y = humans_vel_start_[human_id].second.linear.y;
    start.velocity.angular.z = humans_vel_start_[human_id].second.angular.z;
    // std::cout << "Human Teb Pose: " << 0 << '\n' << start.velocity << '\n';
    start.time_from_start.fromSec(curr_time);

    curr_time += human_teb.TimeDiff(0);

    // intermediate points
    for (int i = 1; i < human_teb_size - 1; ++i) {
      TrajectoryPointMsg &point = human_trajectory[i];
      human_teb.Pose(i).toPoseMsg(point.pose);
      // std::cout << "Human Teb Pose: " << i << '\n' << point.pose.position << '\n';
      point.velocity.linear.z = 0;
      point.velocity.angular.x = point.velocity.angular.y = 0;
      double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
      extractVelocity(human_teb.Pose(i-1), human_teb.Pose(i), human_teb.TimeDiff(i-1), vel1_x, vel1_y, omega1);
      extractVelocity(human_teb.Pose(i), human_teb.Pose(i+1), human_teb.TimeDiff(i), vel2_x, vel2_y, omega2);
      point.velocity.linear.x = 0.5*(vel1_x+vel2_x);
      point.velocity.linear.y = 0.5*(vel1_y+vel2_y);
      point.velocity.angular.z = 0.5 * (omega1 + omega2);
      // std::cout << "Human Teb Pose: " << i << '\n' << point.velocity << '\n';
      point.time_from_start.fromSec(curr_time);

      curr_time += human_teb.TimeDiff(i);
    }
    // goal
    TrajectoryPointMsg &goal = human_trajectory.back();
    // std::cout << "Human Teb Pose: end\n" << goal.pose.position << '\n';

    human_teb.BackPose().toPoseMsg(goal.pose);
    goal.velocity.linear.z = 0;
    goal.velocity.angular.x = goal.velocity.angular.y = 0;
    goal.velocity.linear.x = humans_vel_goal_[human_id].second.linear.x;
    goal.velocity.linear.y = humans_vel_goal_[human_id].second.linear.y;
    goal.velocity.angular.z = humans_vel_goal_[human_id].second.angular.z;
    goal.time_from_start.fromSec(curr_time);
  }
  return;
}
bool TebOptimalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx)
{
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
    look_ahead_idx = teb().sizePoses() - 1;

  for (int i=0; i <= look_ahead_idx; ++i)
  {
    if ( costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
    {
      if (visualization_)
      {
        visualization_->publishInfeasibleRobotPose(teb().Pose(i), *robot_model_);
      }
      return false;
    }
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
    // and interpolates in that case.
    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
    if (i<look_ahead_idx)
    {
      double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i+1).theta()) -
                                              g2o::normalize_theta(teb().Pose(i).theta()));
      Eigen::Vector2d delta_dist = teb().Pose(i+1).position()-teb().Pose(i).position();
      if(fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
      {
        int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular),
                                            std::ceil(delta_dist.norm() / inscribed_radius)) - 1;
        PoseSE2 intermediate_pose = teb().Pose(i);
        for(int step = 0; step < n_additional_samples; ++step)
        {
          intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
          intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() +
                                                           delta_rot / (n_additional_samples + 1.0));
          if ( costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(),
            footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
          {
            if (visualization_)
            {
              visualization_->publishInfeasibleRobotPose(intermediate_pose, *robot_model_);
            }
            return false;
          }
        }
      }
    }
  }
  return true;
}

} // namespace hateb_local_planner
