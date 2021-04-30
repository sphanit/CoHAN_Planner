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

#include <hateb_local_planner/hateb_config.h>

namespace hateb_local_planner
{

void HATebConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh)
{

  nh.param("odom_topic", odom_topic, odom_topic);
  nh.param("map_frame", map_frame, map_frame);

  nh.param("planning_mode", planning_mode, planning_mode);

  // Trajectory
  nh.param("teb_autosize", trajectory.teb_autosize, trajectory.teb_autosize);
  nh.param("dt_ref", trajectory.dt_ref, trajectory.dt_ref);
  nh.param("dt_hysteresis", trajectory.dt_hysteresis, trajectory.dt_hysteresis);
  nh.param("min_samples", trajectory.min_samples, trajectory.min_samples);
  nh.param("max_samples", trajectory.max_samples, trajectory.max_samples);
  nh.param("human_min_samples", trajectory.human_min_samples, trajectory.human_min_samples);
  nh.param("global_plan_overwrite_orientation", trajectory.global_plan_overwrite_orientation, trajectory.global_plan_overwrite_orientation);
  nh.param("allow_init_with_backwards_motion", trajectory.allow_init_with_backwards_motion, trajectory.allow_init_with_backwards_motion);
  nh.getParam("global_plan_via_point_sep", trajectory.global_plan_viapoint_sep); // deprecated, see checkDeprecated()
  if (!nh.param("global_plan_viapoint_sep", trajectory.global_plan_viapoint_sep, trajectory.global_plan_viapoint_sep))
    nh.setParam("global_plan_viapoint_sep", trajectory.global_plan_viapoint_sep); // write deprecated value to param server
  nh.param("via_points_ordered", trajectory.via_points_ordered, trajectory.via_points_ordered);
  nh.param("max_global_plan_lookahead_dist", trajectory.max_global_plan_lookahead_dist, trajectory.max_global_plan_lookahead_dist);
  nh.param("global_plan_prune_distance", trajectory.global_plan_prune_distance, trajectory.global_plan_prune_distance);
  nh.param("exact_arc_length", trajectory.exact_arc_length, trajectory.exact_arc_length);
  nh.param("force_reinit_new_goal_dist", trajectory.force_reinit_new_goal_dist, trajectory.force_reinit_new_goal_dist);
  nh.param("force_reinit_new_goal_angular", trajectory.force_reinit_new_goal_angular, trajectory.force_reinit_new_goal_angular);
  nh.param("feasibility_check_no_poses", trajectory.feasibility_check_no_poses, trajectory.feasibility_check_no_poses);
  nh.param("publish_feedback", trajectory.publish_feedback, trajectory.publish_feedback);
  nh.param("min_resolution_collision_check_angular", trajectory.min_resolution_collision_check_angular, trajectory.min_resolution_collision_check_angular);
  nh.param("control_look_ahead_poses", trajectory.control_look_ahead_poses, trajectory.control_look_ahead_poses);
  nh.param("horizon_reduction_amount", trajectory.horizon_reduction_amount, trajectory.horizon_reduction_amount);
  nh.param("teb_init_skip_dist", trajectory.teb_init_skip_dist, trajectory.teb_init_skip_dist);
  // Robot
  nh.param("max_vel_x", robot.max_vel_x, robot.max_vel_x);
  nh.param("max_vel_x_backwards", robot.max_vel_x_backwards, robot.max_vel_x_backwards);
  nh.param("max_vel_y", robot.max_vel_y, robot.max_vel_y);
  nh.param("max_vel_theta", robot.max_vel_theta, robot.max_vel_theta);
  nh.param("acc_lim_x", robot.acc_lim_x, robot.acc_lim_x);
  nh.param("acc_lim_y", robot.acc_lim_y, robot.acc_lim_y);
  nh.param("acc_lim_theta", robot.acc_lim_theta, robot.acc_lim_theta);
  nh.param("min_turning_radius", robot.min_turning_radius, robot.min_turning_radius);
  nh.param("wheelbase", robot.wheelbase, robot.wheelbase);
  nh.param("cmd_angle_instead_rotvel", robot.cmd_angle_instead_rotvel, robot.cmd_angle_instead_rotvel);
  nh.param("is_footprint_dynamic", robot.is_footprint_dynamic, robot.is_footprint_dynamic);
  nh.param("is_real", robot.is_real, robot.is_real);

  // Human
  nh.param("human_radius", human.radius, human.radius);
  nh.param("max_human_vel_x", human.max_vel_x, human.max_vel_x);
  nh.param("max_human_vel_y", human.max_vel_y, human.max_vel_y);
  nh.param("nominal_human_vel_x", human.nominal_vel_x, human.nominal_vel_x);
  nh.param("max_human_vel_x_backwards", human.max_vel_x_backwards,
           human.max_vel_x_backwards);
  nh.param("max_human_vel_theta", human.max_vel_theta, human.max_vel_theta);
  nh.param("human_acc_lim_x", human.acc_lim_x, human.acc_lim_x);
  nh.param("human_acc_lim_y", human.acc_lim_y, human.acc_lim_y);
  nh.param("human_acc_lim_theta", human.acc_lim_theta, human.acc_lim_theta);
  nh.param("num_moving_avg", human.num_moving_avg, human.num_moving_avg);


  // GoalTolerance
  nh.param("xy_goal_tolerance", goal_tolerance.xy_goal_tolerance, goal_tolerance.xy_goal_tolerance);
  nh.param("yaw_goal_tolerance", goal_tolerance.yaw_goal_tolerance, goal_tolerance.yaw_goal_tolerance);
  nh.param("free_goal_vel", goal_tolerance.free_goal_vel, goal_tolerance.free_goal_vel);
  nh.param("complete_global_plan", goal_tolerance.complete_global_plan, goal_tolerance.complete_global_plan);

  // Obstacles
  nh.param("min_obstacle_dist", obstacles.min_obstacle_dist, obstacles.min_obstacle_dist);
  nh.param("inflation_dist", obstacles.inflation_dist, obstacles.inflation_dist);
  nh.param("dynamic_obstacle_inflation_dist", obstacles.dynamic_obstacle_inflation_dist, obstacles.dynamic_obstacle_inflation_dist);
  nh.param("include_dynamic_obstacles", obstacles.include_dynamic_obstacles, obstacles.include_dynamic_obstacles);
  nh.param("include_costmap_obstacles", obstacles.include_costmap_obstacles, obstacles.include_costmap_obstacles);
  nh.param("costmap_obstacles_behind_robot_dist", obstacles.costmap_obstacles_behind_robot_dist, obstacles.costmap_obstacles_behind_robot_dist);
  nh.param("obstacle_poses_affected", obstacles.obstacle_poses_affected, obstacles.obstacle_poses_affected);
  nh.param("legacy_obstacle_association", obstacles.legacy_obstacle_association, obstacles.legacy_obstacle_association);
  nh.param("obstacle_association_force_inclusion_factor", obstacles.obstacle_association_force_inclusion_factor, obstacles.obstacle_association_force_inclusion_factor);
  nh.param("obstacle_association_cutoff_factor", obstacles.obstacle_association_cutoff_factor, obstacles.obstacle_association_cutoff_factor);
  nh.param("costmap_converter_plugin", obstacles.costmap_converter_plugin, obstacles.costmap_converter_plugin);
  nh.param("costmap_converter_spin_thread", obstacles.costmap_converter_spin_thread, obstacles.costmap_converter_spin_thread);

  // Optimization
  nh.param("no_inner_iterations", optim.no_inner_iterations, optim.no_inner_iterations);
  nh.param("no_outer_iterations", optim.no_outer_iterations, optim.no_outer_iterations);
  nh.param("optimization_activate", optim.optimization_activate, optim.optimization_activate);
  nh.param("optimization_verbose", optim.optimization_verbose, optim.optimization_verbose);
  nh.param("penalty_epsilon", optim.penalty_epsilon, optim.penalty_epsilon);
  nh.param("time_penalty_epsilon", optim.time_penalty_epsilon, optim.time_penalty_epsilon);
  nh.param("cap_optimaltime_penalty", optim.cap_optimaltime_penalty, optim.cap_optimaltime_penalty);
  nh.param("weight_max_vel_x", optim.weight_max_vel_x, optim.weight_max_vel_x);
  nh.param("weight_max_vel_y", optim.weight_max_vel_y, optim.weight_max_vel_y);
  nh.param("weight_max_vel_theta", optim.weight_max_vel_theta, optim.weight_max_vel_theta);
  nh.param("weight_acc_lim_x", optim.weight_acc_lim_x, optim.weight_acc_lim_x);
  nh.param("weight_acc_lim_y", optim.weight_acc_lim_y, optim.weight_acc_lim_y);
  nh.param("weight_acc_lim_theta", optim.weight_acc_lim_theta, optim.weight_acc_lim_theta);
  nh.param("weight_kinematics_nh", optim.weight_kinematics_nh, optim.weight_kinematics_nh);
  nh.param("weight_kinematics_forward_drive", optim.weight_kinematics_forward_drive, optim.weight_kinematics_forward_drive);
  nh.param("weight_kinematics_turning_radius", optim.weight_kinematics_turning_radius, optim.weight_kinematics_turning_radius);
  nh.param("weight_optimaltime", optim.weight_optimaltime, optim.weight_optimaltime);
  nh.param("weight_shortest_path", optim.weight_shortest_path, optim.weight_shortest_path);
  nh.param("weight_obstacle", optim.weight_obstacle, optim.weight_obstacle);
  nh.param("weight_inflation", optim.weight_inflation, optim.weight_inflation);
  nh.param("weight_dynamic_obstacle", optim.weight_dynamic_obstacle, optim.weight_dynamic_obstacle);
  nh.param("weight_dynamic_obstacle_inflation", optim.weight_dynamic_obstacle_inflation, optim.weight_dynamic_obstacle_inflation);
  nh.param("weight_viapoint", optim.weight_viapoint, optim.weight_viapoint);
  nh.param("weight_prefer_rotdir", optim.weight_prefer_rotdir, optim.weight_prefer_rotdir);
  nh.param("weight_adapt_factor", optim.weight_adapt_factor, optim.weight_adapt_factor);
  nh.param("obstacle_cost_exponent", optim.obstacle_cost_exponent, optim.obstacle_cost_exponent);


  nh.param("weight_max_human_vel_x", optim.weight_max_human_vel_x,
           optim.weight_max_human_vel_x);
 nh.param("weight_max_human_vel_y", optim.weight_max_human_vel_y,
           optim.weight_max_human_vel_y);
  nh.param("weight_nominal_human_vel_x", optim.weight_nominal_human_vel_x,
           optim.weight_nominal_human_vel_x);
  nh.param("weight_max_human_vel_theta", optim.weight_max_human_vel_theta,
           optim.weight_max_human_vel_theta);
  nh.param("weight_human_acc_lim_x", optim.weight_acc_lim_x,
           optim.weight_human_acc_lim_x);
  nh.param("weight_human_acc_lim_y", optim.weight_acc_lim_y,
            optim.weight_human_acc_lim_y);
  nh.param("weight_human_acc_lim_theta", optim.weight_acc_lim_theta,
           optim.weight_human_acc_lim_theta);
  nh.param("weight_human_optimaltime", optim.weight_human_optimaltime,
           optim.weight_human_optimaltime);
  nh.param("weight_human_viapoint", optim.weight_human_viapoint,
           optim.weight_human_viapoint);
  nh.param("weight_human_robot_safety", optim.weight_human_robot_safety,
           optim.weight_human_robot_safety);
  nh.param("weight_human_human_safety", optim.weight_human_human_safety,
           optim.weight_human_human_safety);
  nh.param("weight_human_robot_ttc", optim.weight_human_robot_ttc,
           optim.weight_human_robot_ttc);
  nh.param("weight_human_robot_ttcplus", optim.weight_human_robot_ttcplus,
          optim.weight_human_robot_ttcplus);
  nh.param("weight_human_robot_rel_vel", optim.weight_human_robot_rel_vel,
           optim.weight_human_robot_rel_vel);
  nh.param("human_robot_ttc_scale_alpha", optim.human_robot_ttc_scale_alpha,
           optim.human_robot_ttc_scale_alpha);
  nh.param("human_robot_ttcplus_scale_alpha", optim.human_robot_ttcplus_scale_alpha,
           optim.human_robot_ttcplus_scale_alpha);
  nh.param("weight_human_robot_visibility", optim.weight_human_robot_visibility,
           optim.weight_human_robot_visibility);
  nh.param("disable_warm_start", optim.disable_warm_start,
           optim.disable_warm_start);
  nh.param("disable_rapid_omega_chage", optim.disable_rapid_omega_chage,
           optim.disable_rapid_omega_chage);
  nh.param("omega_chage_time_seperation", optim.omega_chage_time_seperation,
           optim.omega_chage_time_seperation);

  // Hateb
  nh.param("use_human_robot_safety_c", hateb.use_human_robot_safety_c,
           hateb.use_human_robot_safety_c);
  nh.param("use_human_human_safety_c", hateb.use_human_human_safety_c,
           hateb.use_human_human_safety_c);
  nh.param("use_human_robot_ttc_c", hateb.use_human_robot_ttc_c,
           hateb.use_human_robot_ttc_c);
  nh.param("use_human_robot_ttcplus_c", hateb.use_human_robot_ttcplus_c,
           hateb.use_human_robot_ttcplus_c);
  nh.param("scale_human_robot_ttc_c", hateb.scale_human_robot_ttc_c,
           hateb.scale_human_robot_ttc_c);
  nh.param("scale_human_robot_ttcplus_c", hateb.scale_human_robot_ttcplus_c,
           hateb.scale_human_robot_ttcplus_c);
  nh.param("use_human_robot_rel_vel_c", hateb.use_human_robot_rel_vel_c,
           hateb.use_human_robot_rel_vel_c);
  nh.param("use_human_robot_visi_c", hateb.use_human_robot_visi_c,
           hateb.use_human_robot_visi_c);
  nh.param("use_human_elastic_vel", hateb.use_human_elastic_vel,
           hateb.use_human_elastic_vel);
  nh.param("use_external_prediction", hateb.use_external_prediction,
            hateb.use_external_prediction);
  nh.param("predict_human_behind_robot", hateb.predict_human_behind_robot,
            hateb.predict_human_behind_robot);
  nh.param("predict_human_goal", hateb.predict_human_goal,
            hateb.predict_human_goal);
  nh.param("min_human_robot_dist", hateb.min_human_robot_dist,
                     hateb.min_human_robot_dist);
  nh.param("min_human_human_dist", hateb.min_human_human_dist,
                     hateb.min_human_human_dist);
  nh.param("ttc_threshold", hateb.ttc_threshold, hateb.ttc_threshold);
  nh.param("human_pose_prediction_reset_time", hateb.pose_prediction_reset_time,
            hateb.pose_prediction_reset_time);
  nh.param("rel_vel_cost_threshold", hateb.rel_vel_cost_threshold, hateb.rel_vel_cost_threshold);
  nh.param("ttcplus_threshold", hateb.ttcplus_threshold, hateb.ttcplus_threshold);
  nh.param("ttcplus_timer", hateb.ttcplus_timer, hateb.ttcplus_timer);

  // Homotopy Class Planner
  nh.param("enable_homotopy_class_planning", hcp.enable_homotopy_class_planning, hcp.enable_homotopy_class_planning);
  nh.param("enable_multithreading", hcp.enable_multithreading, hcp.enable_multithreading);
  nh.param("simple_exploration", hcp.simple_exploration, hcp.simple_exploration);
  nh.param("max_number_classes", hcp.max_number_classes, hcp.max_number_classes);
  nh.param("selection_obst_cost_scale", hcp.selection_obst_cost_scale, hcp.selection_obst_cost_scale);
  nh.param("selection_prefer_initial_plan", hcp.selection_prefer_initial_plan, hcp.selection_prefer_initial_plan);
  nh.param("selection_viapoint_cost_scale", hcp.selection_viapoint_cost_scale, hcp.selection_viapoint_cost_scale);
  nh.param("selection_cost_hysteresis", hcp.selection_cost_hysteresis, hcp.selection_cost_hysteresis);
  nh.param("selection_alternative_time_cost", hcp.selection_alternative_time_cost, hcp.selection_alternative_time_cost);
  nh.param("switching_blocking_period", hcp.switching_blocking_period, hcp.switching_blocking_period);
  nh.param("roadmap_graph_samples", hcp.roadmap_graph_no_samples, hcp.roadmap_graph_no_samples);
  nh.param("roadmap_graph_area_width", hcp.roadmap_graph_area_width, hcp.roadmap_graph_area_width);
  nh.param("roadmap_graph_area_length_scale", hcp.roadmap_graph_area_length_scale, hcp.roadmap_graph_area_length_scale);
  nh.param("h_signature_prescaler", hcp.h_signature_prescaler, hcp.h_signature_prescaler);
  nh.param("h_signature_threshold", hcp.h_signature_threshold, hcp.h_signature_threshold);
  nh.param("obstacle_keypoint_offset", hcp.obstacle_keypoint_offset, hcp.obstacle_keypoint_offset);
  nh.param("obstacle_heading_threshold", hcp.obstacle_heading_threshold, hcp.obstacle_heading_threshold);
  nh.param("viapoints_all_candidates", hcp.viapoints_all_candidates, hcp.viapoints_all_candidates);
  nh.param("visualize_hc_graph", hcp.visualize_hc_graph, hcp.visualize_hc_graph);
  nh.param("visualize_with_time_as_z_axis_scale", hcp.visualize_with_time_as_z_axis_scale, hcp.visualize_with_time_as_z_axis_scale);
  nh.param("delete_detours_backwards", hcp.delete_detours_backwards, hcp.delete_detours_backwards);
  nh.param("detours_orientation_tolerance", hcp.detours_orientation_tolerance, hcp.detours_orientation_tolerance);
  nh.param("length_start_orientation_vector", hcp.length_start_orientation_vector, hcp.length_start_orientation_vector);
  nh.param("max_ratio_detours_duration_best_duration", hcp.max_ratio_detours_duration_best_duration, hcp.max_ratio_detours_duration_best_duration);

  // Recovery
  nh.param("shrink_horizon_backup", recovery.shrink_horizon_backup, recovery.shrink_horizon_backup);
  nh.param("shrink_horizon_min_duration", recovery.shrink_horizon_min_duration, recovery.shrink_horizon_min_duration);
  nh.param("oscillation_recovery", recovery.oscillation_recovery, recovery.oscillation_recovery);
  nh.param("oscillation_v_eps", recovery.oscillation_v_eps, recovery.oscillation_v_eps);
  nh.param("oscillation_omega_eps", recovery.oscillation_omega_eps, recovery.oscillation_omega_eps);
  nh.param("oscillation_recovery_min_duration", recovery.oscillation_recovery_min_duration, recovery.oscillation_recovery_min_duration);
  nh.param("oscillation_filter_duration", recovery.oscillation_filter_duration, recovery.oscillation_filter_duration);

  // Visualization
  nh.param("publish_robot_global_plan", visualization.publish_robot_global_plan,
           visualization.publish_robot_global_plan);
  nh.param("publish_robot_local_plan", visualization.publish_robot_local_plan,
           visualization.publish_robot_local_plan);
  nh.param("publish_robot_local_plan_poses",
           visualization.publish_robot_local_plan_poses,
           visualization.publish_robot_local_plan_poses);
  nh.param("publish_robot_local_plan_fp_poses",
           visualization.publish_robot_local_plan_fp_poses,
           visualization.publish_robot_local_plan_fp_poses);
  nh.param("publish_human_global_plans",
           visualization.publish_human_global_plans,
           visualization.publish_human_global_plans);
  nh.param("publish_human_local_plans", visualization.publish_human_local_plans,
           visualization.publish_human_local_plans);
  nh.param("publish_human_local_plan_poses",
           visualization.publish_human_local_plan_poses,
           visualization.publish_human_local_plan_poses);
  nh.param("publish_human_local_plan_fp_poses",
           visualization.publish_human_local_plan_fp_poses,
           visualization.publish_human_local_plan_fp_poses);
  nh.param("pose_array_z_scale", visualization.pose_array_z_scale,
           visualization.pose_array_z_scale);

  // approach
  nh.param("approach_id", approach.approach_id, approach.approach_id);
  nh.param("approach_dist", approach.approach_dist, approach.approach_dist);
  nh.param("approach_angle", approach.approach_angle, approach.approach_angle);
  nh.param("approach_dist_tolerance", approach.approach_dist_tolerance,
           approach.approach_dist_tolerance);
  nh.param("approach_angle_tolerance", approach.approach_angle_tolerance,
           approach.approach_angle_tolerance);

  checkParameters();
  checkDeprecated(nh);
}

void HATebConfig::reconfigure(HATebLocalPlannerReconfigureConfig& cfg)
{
  boost::mutex::scoped_lock l(config_mutex_);

  planning_mode = cfg.planning_mode;

  // Trajectory
  trajectory.teb_autosize = cfg.teb_autosize;
  trajectory.dt_ref = cfg.dt_ref;
  trajectory.dt_hysteresis = cfg.dt_hysteresis;
  trajectory.global_plan_overwrite_orientation = cfg.global_plan_overwrite_orientation;
  trajectory.allow_init_with_backwards_motion = cfg.allow_init_with_backwards_motion;
  trajectory.global_plan_viapoint_sep = cfg.global_plan_viapoint_sep;
  trajectory.via_points_ordered = cfg.via_points_ordered;
  trajectory.max_global_plan_lookahead_dist = cfg.max_global_plan_lookahead_dist;
  trajectory.exact_arc_length = cfg.exact_arc_length;
  trajectory.force_reinit_new_goal_dist = cfg.force_reinit_new_goal_dist;
  trajectory.force_reinit_new_goal_angular = cfg.force_reinit_new_goal_angular;
  trajectory.feasibility_check_no_poses = cfg.feasibility_check_no_poses;
  trajectory.publish_feedback = cfg.publish_feedback;
  trajectory.horizon_reduction_amount = cfg.horizon_reduction_amount;
  trajectory.teb_init_skip_dist = cfg.teb_init_skip_dist;

  // Robot
  robot.is_real = cfg.is_real;
  robot.max_vel_x = cfg.max_vel_x;
  robot.max_vel_x_backwards = cfg.max_vel_x_backwards;
  robot.max_vel_y = cfg.max_vel_y;
  robot.max_vel_theta = cfg.max_vel_theta;
  robot.acc_lim_x = cfg.acc_lim_x;
  robot.acc_lim_y = cfg.acc_lim_y;
  robot.acc_lim_theta = cfg.acc_lim_theta;
  robot.min_turning_radius = cfg.min_turning_radius;
  robot.wheelbase = cfg.wheelbase;
  robot.cmd_angle_instead_rotvel = cfg.cmd_angle_instead_rotvel;

  // Human
  human.max_vel_x = cfg.max_human_vel_x;
  human.max_vel_y = cfg.max_human_vel_y;
  human.nominal_vel_x = cfg.nominal_human_vel_x;
  human.max_vel_x_backwards = cfg.max_human_vel_x_backwards;
  human.max_vel_theta = cfg.max_human_vel_theta;
  human.acc_lim_x = cfg.human_acc_lim_x;
  human.acc_lim_y = cfg.human_acc_lim_y;
  human.acc_lim_theta = cfg.human_acc_lim_theta;
  human.fov = cfg.fov;
  human.num_moving_avg = cfg.num_moving_avg;

  // GoalTolerance
  goal_tolerance.xy_goal_tolerance = cfg.xy_goal_tolerance;
  goal_tolerance.yaw_goal_tolerance = cfg.yaw_goal_tolerance;
  goal_tolerance.free_goal_vel = cfg.free_goal_vel;

  // Obstacles
  obstacles.min_obstacle_dist = cfg.min_obstacle_dist;
  obstacles.inflation_dist = cfg.inflation_dist;
  obstacles.legacy_obstacle_association = cfg.legacy_obstacle_association;
  obstacles.obstacle_association_force_inclusion_factor = cfg.obstacle_association_force_inclusion_factor;
  obstacles.obstacle_association_cutoff_factor = cfg.obstacle_association_cutoff_factor;
  obstacles.use_nonlinear_obstacle_penalty = cfg.use_nonlinear_obstacle_penalty;
  obstacles.obstacle_cost_mult = cfg.obstacle_cost_mult;
  obstacles.dynamic_obstacle_inflation_dist = cfg.dynamic_obstacle_inflation_dist;
  obstacles.include_dynamic_obstacles = cfg.include_dynamic_obstacles;
  obstacles.include_costmap_obstacles = cfg.include_costmap_obstacles;
  obstacles.costmap_obstacles_behind_robot_dist = cfg.costmap_obstacles_behind_robot_dist;
  obstacles.obstacle_poses_affected = cfg.obstacle_poses_affected;


  // Optimization
  optim.no_inner_iterations = cfg.no_inner_iterations;
  optim.no_outer_iterations = cfg.no_outer_iterations;
  optim.optimization_activate = cfg.optimization_activate;
  optim.optimization_verbose = cfg.optimization_verbose;
  optim.penalty_epsilon = cfg.penalty_epsilon;
  optim.time_penalty_epsilon = cfg.time_penalty_epsilon;
  optim.cap_optimaltime_penalty = cfg.cap_optimaltime_penalty;
  optim.weight_max_vel_x = cfg.weight_max_vel_x;
  optim.weight_max_vel_y = cfg.weight_max_vel_y;
  optim.weight_max_vel_theta = cfg.weight_max_vel_theta;
  optim.weight_acc_lim_x = cfg.weight_acc_lim_x;
  optim.weight_acc_lim_y = cfg.weight_acc_lim_y;
  optim.weight_acc_lim_theta = cfg.weight_acc_lim_theta;
  optim.weight_kinematics_nh = cfg.weight_kinematics_nh;
  optim.weight_kinematics_forward_drive = cfg.weight_kinematics_forward_drive;
  optim.weight_kinematics_turning_radius = cfg.weight_kinematics_turning_radius;
  optim.weight_optimaltime = cfg.weight_optimaltime;
  optim.weight_shortest_path = cfg.weight_shortest_path;
  optim.weight_obstacle = cfg.weight_obstacle;
  optim.weight_inflation = cfg.weight_inflation;
  optim.weight_dynamic_obstacle = cfg.weight_dynamic_obstacle;
  optim.weight_dynamic_obstacle_inflation = cfg.weight_dynamic_obstacle_inflation;
  optim.weight_viapoint = cfg.weight_viapoint;
  optim.weight_adapt_factor = cfg.weight_adapt_factor;
  optim.obstacle_cost_exponent = cfg.obstacle_cost_exponent;

  optim.weight_max_human_vel_x = cfg.weight_max_human_vel_x;
  optim.weight_max_human_vel_y = cfg.weight_max_human_vel_y;
  optim.weight_nominal_human_vel_x = cfg.weight_nominal_human_vel_x;
  optim.weight_max_human_vel_theta = cfg.weight_max_human_vel_theta;
  optim.weight_human_acc_lim_x = cfg.weight_human_acc_lim_x;
  optim.weight_human_acc_lim_y = cfg.weight_human_acc_lim_y;
  optim.weight_human_acc_lim_theta = cfg.weight_human_acc_lim_theta;
  optim.weight_human_optimaltime = cfg.weight_human_optimaltime;
  optim.weight_human_viapoint = cfg.weight_human_viapoint;
  optim.weight_human_robot_safety = cfg.weight_human_robot_safety;
  optim.weight_human_human_safety = cfg.weight_human_human_safety;
  optim.weight_human_robot_ttc = cfg.weight_human_robot_ttc;
  optim.weight_human_robot_ttcplus = cfg.weight_human_robot_ttcplus;
  optim.weight_human_robot_rel_vel = cfg.weight_human_robot_rel_vel;
  optim.weight_human_robot_visibility = cfg.weight_human_robot_visibility;
  optim.human_robot_ttc_scale_alpha = cfg.human_robot_ttc_scale_alpha;
  optim.human_robot_ttcplus_scale_alpha = cfg.human_robot_ttcplus_scale_alpha;
  optim.disable_warm_start = cfg.disable_warm_start;
  optim.disable_rapid_omega_chage = cfg.disable_rapid_omega_chage;
  optim.omega_chage_time_seperation = cfg.omega_chage_time_seperation;

  //Hateb
  hateb.use_human_robot_safety_c = cfg.use_human_robot_safety_c;
  hateb.use_human_human_safety_c = cfg.use_human_human_safety_c;
  hateb.use_human_robot_ttc_c = cfg.use_human_robot_ttc_c;
  hateb.use_human_robot_ttcplus_c = cfg.use_human_robot_ttcplus_c;
  hateb.scale_human_robot_ttc_c = cfg.scale_human_robot_ttc_c;
  hateb.scale_human_robot_ttcplus_c = cfg.scale_human_robot_ttcplus_c;
  hateb.use_human_robot_rel_vel_c = cfg.use_human_robot_rel_vel_c;
  hateb.use_human_robot_visi_c = cfg.use_human_robot_visi_c;
  hateb.use_human_elastic_vel = cfg.use_human_elastic_vel;
  hateb.use_external_prediction = cfg.use_external_prediction;
  hateb.predict_human_behind_robot = cfg.predict_human_behind_robot;
  hateb.predict_human_goal = cfg.predict_human_goal;
  hateb.min_human_robot_dist = cfg.min_human_robot_dist;
  hateb.min_human_human_dist = cfg.min_human_human_dist;
  hateb.ttc_threshold = cfg.ttc_threshold;
  hateb.ttcplus_threshold = cfg.ttcplus_threshold;
  hateb.ttcplus_timer = cfg.ttcplus_timer;
  hateb.rel_vel_cost_threshold = cfg.rel_vel_cost_threshold;
  hateb.visibility_cost_threshold = cfg.visibility_cost_threshold;
  hateb.pose_prediction_reset_time = cfg.human_pose_prediction_reset_time;

  // Homotopy Class Planner
  hcp.enable_multithreading = cfg.enable_multithreading;
  hcp.max_number_classes = cfg.max_number_classes;
  hcp.selection_cost_hysteresis = cfg.selection_cost_hysteresis;
  hcp.selection_prefer_initial_plan = cfg.selection_prefer_initial_plan;
  hcp.selection_obst_cost_scale = cfg.selection_obst_cost_scale;
  hcp.selection_viapoint_cost_scale = cfg.selection_viapoint_cost_scale;
  hcp.selection_alternative_time_cost = cfg.selection_alternative_time_cost;
  hcp.switching_blocking_period = cfg.switching_blocking_period;

  hcp.obstacle_heading_threshold = cfg.obstacle_heading_threshold;
  hcp.roadmap_graph_no_samples = cfg.roadmap_graph_no_samples;
  hcp.roadmap_graph_area_width = cfg.roadmap_graph_area_width;
  hcp.roadmap_graph_area_length_scale = cfg.roadmap_graph_area_length_scale;
  hcp.h_signature_prescaler = cfg.h_signature_prescaler;
  hcp.h_signature_threshold = cfg.h_signature_threshold;
  hcp.viapoints_all_candidates = cfg.viapoints_all_candidates;
  hcp.visualize_hc_graph = cfg.visualize_hc_graph;
  hcp.visualize_with_time_as_z_axis_scale = cfg.visualize_with_time_as_z_axis_scale;

  // Recovery

  recovery.shrink_horizon_backup = cfg.shrink_horizon_backup;
  recovery.oscillation_recovery = cfg.oscillation_recovery;

  // Visualization
  visualization.publish_robot_global_plan = cfg.publish_robot_global_plan;
  visualization.publish_robot_local_plan = cfg.publish_robot_local_plan;
  visualization.publish_robot_local_plan_poses =
      cfg.publish_robot_local_plan_poses;
  visualization.publish_robot_local_plan_fp_poses =
      cfg.publish_robot_local_plan_fp_poses;
  visualization.publish_human_global_plans = cfg.publish_human_global_plans;
  visualization.publish_human_local_plans = cfg.publish_human_local_plans;
  visualization.publish_human_local_plan_poses =
      cfg.publish_human_local_plan_poses;
  visualization.publish_human_local_plan_fp_poses =
      cfg.publish_human_local_plan_fp_poses;
  visualization.pose_array_z_scale = cfg.pose_array_z_scale;

  // approach
  approach.approach_id = cfg.approach_id;
  approach.approach_dist = cfg.approach_dist;
  approach.approach_angle = cfg.approach_angle;
  approach.approach_dist_tolerance = cfg.approach_dist_tolerance;
  approach.approach_angle_tolerance = cfg.approach_angle_tolerance;

  checkParameters();
}

void HATebConfig::checkParameters() const
{
  // positive backward velocity?
  if (robot.max_vel_x_backwards <= 0)
    ROS_WARN("HATebLocalPlannerROS() Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");

  // bounds smaller than penalty epsilon
  if (robot.max_vel_x <= optim.penalty_epsilon)
    ROS_WARN("HATebLocalPlannerROS() Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  if (robot.max_vel_x_backwards <= optim.penalty_epsilon)
    ROS_WARN("HATebLocalPlannerROS() Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  if (robot.max_vel_theta <= optim.penalty_epsilon)
    ROS_WARN("HATebLocalPlannerROS() Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  if (robot.acc_lim_x <= optim.penalty_epsilon)
    ROS_WARN("HATebLocalPlannerROS() Param Warning: acc_lim_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  if (robot.acc_lim_theta <= optim.penalty_epsilon)
    ROS_WARN("HATebLocalPlannerROS() Param Warning: acc_lim_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  // dt_ref and dt_hyst
  if (trajectory.dt_ref <= trajectory.dt_hysteresis)
    ROS_WARN("HATebLocalPlannerROS() Param Warning: dt_ref <= dt_hysteresis. The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at least one of them!");

  // min number of samples
  if (trajectory.min_samples <3)
    ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't enough degrees of freedom to plan a trajectory for you. Please increase ...");

  // costmap obstacle behind robot
  if (obstacles.costmap_obstacles_behind_robot_dist < 0)
    ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter 'costmap_obstacles_behind_robot_dist' should be positive or zero.");

  // hcp: obstacle heading threshold
  if (hcp.obstacle_keypoint_offset>=1 || hcp.obstacle_keypoint_offset<=0)
    ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter obstacle_heading_threshold must be in the interval ]0,1[. 0=0deg opening angle, 1=90deg opening angle.");

  // carlike
  if (robot.cmd_angle_instead_rotvel && robot.wheelbase==0)
    ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but wheelbase is set to zero: undesired behavior.");

  if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius==0)
    ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a diffdrive robot");

  // positive weight_adapt_factor
  if (optim.weight_adapt_factor < 1.0)
      ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter weight_adapt_factor shoud be >= 1.0");

  if (recovery.oscillation_filter_duration < 0)
      ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter oscillation_filter_duration must be >= 0");

  // weights
  if (optim.weight_optimaltime <= 0)
      ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter weight_optimaltime shoud be > 0 (even if weight_shortest_path is in use)");

}

void HATebConfig::checkDeprecated(const ros::NodeHandle& nh) const
{
  if (nh.hasParam("line_obstacle_poses_affected") || nh.hasParam("polygon_obstacle_poses_affected"))
    ROS_WARN("HATebLocalPlannerROS() Param Warning: 'line_obstacle_poses_affected' and 'polygon_obstacle_poses_affected' are deprecated. They share now the common parameter 'obstacle_poses_affected'.");

  if (nh.hasParam("weight_point_obstacle") || nh.hasParam("weight_line_obstacle") || nh.hasParam("weight_poly_obstacle"))
    ROS_WARN("HATebLocalPlannerROS() Param Warning: 'weight_point_obstacle', 'weight_line_obstacle' and 'weight_poly_obstacle' are deprecated. They are replaced by the single param 'weight_obstacle'.");

  if (nh.hasParam("costmap_obstacles_front_only"))
    ROS_WARN("HATebLocalPlannerROS() Param Warning: 'costmap_obstacles_front_only' is deprecated. It is replaced by 'costmap_obstacles_behind_robot_dist' to define the actual area taken into account.");

  if (nh.hasParam("costmap_emergency_stop_dist"))
    ROS_WARN("HATebLocalPlannerROS() Param Warning: 'costmap_emergency_stop_dist' is deprecated. You can safely remove it from your parameter config.");

  if (nh.hasParam("alternative_time_cost"))
    ROS_WARN("HATebLocalPlannerROS() Param Warning: 'alternative_time_cost' is deprecated. It has been replaced by 'selection_alternative_time_cost'.");

  if (nh.hasParam("global_plan_via_point_sep"))
    ROS_WARN("HATebLocalPlannerROS() Param Warning: 'global_plan_via_point_sep' is deprecated. It has been replaced by 'global_plan_viapoint_sep' due to consistency reasons.");
}


} // namespace hateb_local_planner
