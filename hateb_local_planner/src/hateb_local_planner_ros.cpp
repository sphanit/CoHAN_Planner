/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Copyright (c) 2020 LAAS/CNRS
 *  All rights reserved.
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

#define PREDICT_SERVICE_NAME "/human_path_predict/predict_human_poses"
#define RESET_PREDICTION_SERVICE_NAME "/human_path_predict/reset_external_paths"
#define PUBLISH_MARKERS_SRV_NAME "/human_path_predict/publish_prediction_markers"
#define HUMAN_GOAL_SRV_NAME "/human_path_predict/check_human_goal"
#define HUMANS_SUB_TOPIC "/tracked_humans"
#define OPTIMIZE_SRV_NAME "optimize"
#define APPROACH_SRV_NAME "set_approach_id"
#define HATEB_LOG "hateb_log"
// #define OP_COSTS_TOPIC "optimization_costs"
// #define ROB_POS_TOPIC "Robot_Pose"
#define DEFAULT_HUMAN_SEGMENT human_msgs::TrackedSegmentType::TORSO
#define THROTTLE_RATE 5.0 // seconds

#include <hateb_local_planner/hateb_local_planner_ros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/algorithm/string.hpp>
// MBF return codes
#include <mbf_msgs/ExePathResult.h>
#include <std_msgs/Float64.h>

// pluginlib macros
#include <pluginlib/class_list_macros.h>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include <string>

// register this planner both as a BaseLocalPlanner and as a MBF's CostmapController plugin
PLUGINLIB_EXPORT_CLASS(hateb_local_planner::HATebLocalPlannerROS, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(hateb_local_planner::HATebLocalPlannerROS, mbf_costmap_core::CostmapController)

namespace hateb_local_planner
{


HATebLocalPlannerROS::HATebLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), costmap_model_(NULL),
                                           costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                           dynamic_recfg_(NULL), custom_via_points_active_(false), goal_reached_(false), no_infeasible_plans_(0),
                                           last_preferred_rotdir_(RotType::none),horizon_reduced_(false), initialized_(false)
{
}

HATebLocalPlannerROS::~HATebLocalPlannerROS()
{
}

void HATebLocalPlannerROS::reconfigureCB(HATebLocalPlannerReconfigureConfig& config, uint32_t level)
{
  cfg_.reconfigure(config);
}

void HATebLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  // check if the plugin is already initialized
  if(!initialized_)
  {
    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle nh("~/" + name);

    // get parameters of TebConfig via the nodehandle and override the default config
    cfg_.loadRosParamFromNodeHandle(nh);

    // reserve some memory for obstacles
    obstacles_.reserve(500);

    // init some variables
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

    costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);
    global_frame_ = costmap_ros_->getGlobalFrameID();
    cfg_.map_frame = global_frame_; // TODO
    robot_base_frame_ = costmap_ros_->getBaseFrameID();
    // create visualization instance
    visualization_ = TebVisualizationPtr(new TebVisualization(nh, cfg_));

    // create robot footprint/contour model for optimization
    RobotFootprintModelPtr robot_model = getRobotFootprintFromParamServer(nh);

    CircularRobotFootprintPtr human_model = NULL;
    auto human_radius = cfg_.human.radius;
    if (human_radius < 0.0) {
      ROS_WARN("human radius is set to negative, using 0.0");
      human_radius = 0.0;
    }
    human_model = boost::make_shared<CircularRobotFootprint>(human_radius);

    // create the planner instance
    if (cfg_.hcp.enable_homotopy_class_planning)
    {
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_, human_model, &humans_via_points_map_));
      ROS_INFO("Parallel planning in distinctive topologies enabled.");
    }
    else
    {
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_, human_model, &humans_via_points_map_));
      planner_->local_weight_optimaltime_ = cfg_.optim.weight_optimaltime;
      ROS_INFO("Parallel planning in distinctive topologies disabled.");
    }



    //Initialize a costmap to polygon converter
    if (!cfg_.obstacles.costmap_converter_plugin.empty())
    {
      try
      {
        costmap_converter_ = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
        std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin);
        // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
        boost::replace_all(converter_name, "::", "/");
        costmap_converter_->setOdomTopic(cfg_.odom_topic);
        costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
        costmap_converter_->setCostmap2D(costmap_);

        costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_, cfg_.obstacles.costmap_converter_spin_thread);
        ROS_INFO_STREAM("Costmap conversion plugin " << cfg_.obstacles.costmap_converter_plugin << " loaded.");
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
        costmap_converter_.reset();
      }
    }
    else
      ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");


    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);

    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(cfg_.odom_topic);

    // setup dynamic reconfigure
    dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<HATebLocalPlannerReconfigureConfig> >(nh);
    dynamic_reconfigure::Server<HATebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&HATebLocalPlannerROS::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);

    // validate optimization footprint and costmap footprint
    validateFootprints(robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);

    // setup callback for custom obstacles
    custom_obst_sub_ = nh.subscribe("obstacles", 1, &HATebLocalPlannerROS::customObstacleCB, this);

    // setup callback for custom via-points
    via_points_sub_ = nh.subscribe("via_points", 1, &HATebLocalPlannerROS::customViaPointsCB, this);

    // initialize failure detector
    ros::NodeHandle nh_move_base("~");
    double controller_frequency = 10;
    nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
    failure_detector_.setBufferLength(std::round(cfg_.recovery.oscillation_filter_duration*controller_frequency));
    backoff_recovery_.initialize(costmap_ros,cfg_.robot.is_real);

    // setup human prediction client with persistent connection
    predict_humans_client_ = nh.serviceClient<human_path_prediction::HumanPosePredict>(PREDICT_SERVICE_NAME, true);
    reset_humans_prediction_client_ = nh.serviceClient<std_srvs::Empty>(RESET_PREDICTION_SERVICE_NAME, true);
    publish_predicted_markers_client_ = nh.serviceClient<std_srvs::SetBool>(PUBLISH_MARKERS_SRV_NAME, true);
    human_goal_client_ = nh.serviceClient<std_srvs::Trigger>(HUMAN_GOAL_SRV_NAME);

    optimize_server_ = nh.advertiseService(OPTIMIZE_SRV_NAME, &HATebLocalPlannerROS::optimizeStandalone, this);
    approach_server_ = nh.advertiseService(APPROACH_SRV_NAME, &HATebLocalPlannerROS::setApproachID, this);


    humans_sub_ = nh.subscribe(HUMANS_SUB_TOPIC, 1, &HATebLocalPlannerROS::humansCB, this);

    // op_costs_pub_ = nh.advertise<hateb_local_planner::OptimizationCostArray>( OP_COSTS_TOPIC, 1);
    // robot_pose_pub_ = nh.advertise<geometry_msgs::Pose>(ROB_POS_TOPIC, 1);
    humans_states_pub_ = nh.advertise<human_msgs::StateArray>("humans_states",1);
    log_pub_ = nh.advertise<std_msgs::String>(HATEB_LOG,1);

    last_call_time_ = ros::Time::now() - ros::Duration(cfg_.hateb.pose_prediction_reset_time);

    last_omega_sign_change_ = ros::Time::now() - ros::Duration(cfg_.optim.omega_chage_time_seperation);

    last_omega_ = 0.0;
    isDistunderThreshold = false;
    isDistMax = true;
    change_mode = 0;
    isMode = 0;
    stuck = false;
    human_still.clear();
    ext_goal = false;
    backed_off = false;
    goal_ctrl = true;
    humans_states_.states.clear();
    reset_states = true;
    stuck_human_id = -1;

    // set initialized flag
    initialized_ = true;

    ROS_DEBUG("hateb_local_planner plugin initialized.");
  }
  else
  {
    ROS_WARN("hateb_local_planner has already been initialized, doing nothing.");
  }
}



bool HATebLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // check if plugin is initialized
  if(!initialized_)
  {
    ROS_ERROR("hateb_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
  // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

  // reset goal_reached_ flag
  goal_reached_ = false;

  return true;
}

void  HATebLocalPlannerROS::humansCB(const human_msgs::TrackedHumans &tracked_humans){
  //Code block for changing dynamic reconfigure
  // dynamic_reconfigure::ReconfigureRequest srv_req;
  // dynamic_reconfigure::ReconfigureResponse srv_resp;
  // dynamic_reconfigure::DoubleParameter double_param;
  // dynamic_reconfigure::IntParameter int_param;
  // dynamic_reconfigure::Config conf;
  //
  // double_param.name = "nominal_human_vel_x";
  // double_param.value = 0.5;
  // conf.doubles.push_back(double_param);
  //
  // srv_req.config = conf;
  // ros::service::call("/move_base_node/HATebLocalPlannerROS/", srv_req, srv_resp);
  // system("rosrun dynamic_reconfigure dynparam set /move_base_node/HATebLocalPlannerROS/ nominal_human_vel_x 0.5");


  tracked_humans_ = tracked_humans;
  std::vector<double> human_dists;
  std::vector<double> humans_behind;
  // std::vector<bool> humans_behind_ids;

  geometry_msgs::TransformStamped transformStamped;
  transformStamped = tf_->lookupTransform("map", "base_link",ros::Time(0),ros::Duration(0.5));
  auto xpos = transformStamped.transform.translation.x;
  auto ypos = transformStamped.transform.translation.y;
  auto ryaw = tf2::getYaw(transformStamped.transform.rotation);
  Eigen::Vector2d robot_vec(std::cos(ryaw),std::sin(ryaw));
  std::vector<double> hum_xpos;
  std::vector<double> hum_ypos;

  int itr = 0;

  for(auto &human: tracked_humans_.humans){
    if(humans_states_.states.size()<tracked_humans_.humans.size()){
      humans_states_.states.push_back(hateb_local_planner::HumanState::NO_STATE);
      //humans_states_.states.push_back(hateb_local_planner::HumanState::STATIC);
    }
    if(human_vels.size()< human.track_id){
      std::vector<double> h_vels;
      human_vels.push_back(h_vels);
      human_nominal_vels.push_back(0.0);
    }
    for (auto &segment : human.segments){
      if(segment.type==DEFAULT_HUMAN_SEGMENT){
        Eigen::Vector2d rh_vec(segment.pose.pose.position.x-xpos,segment.pose.pose.position.y-ypos);

        humans_behind.push_back(rh_vec.dot(robot_vec));
        human_dists.push_back(rh_vec.norm());

        human_vels[itr].push_back(std::hypot(segment.twist.twist.linear.x, segment.twist.twist.linear.y));

        if((abs(segment.twist.twist.linear.x)+abs(segment.twist.twist.linear.y)+abs(segment.twist.twist.angular.z)) > 0.0001){
          if(humans_states_.states[itr]!=hateb_local_planner::HumanState::BLOCKED){
            humans_states_.states[itr] = hateb_local_planner::HumanState::MOVING;
          }
        }

        auto n = human_vels[itr].size();
        float average = 0.0f;
        if (n != 0) {
          average = accumulate(human_vels[itr].begin(), human_vels[itr].end(), 0.0) / n;
        }
        human_nominal_vels[itr] = average;

        if(n==cfg_.human.num_moving_avg)
          human_vels[itr].erase(human_vels[itr].begin());
        }
    }
    itr++;
  }
  ROS_INFO_ONCE("Number of humans, %d ", (int)human_vels.size());

  human_still.clear();
  for(int i=0;i<prev_tracked_humans_.humans.size();i++){
    for (int j=0;j<prev_tracked_humans_.humans[i].segments.size();j++){
      if(prev_tracked_humans_.humans[i].segments[j].type==DEFAULT_HUMAN_SEGMENT){
        double hum_move_dist = std::hypot(tracked_humans_.humans[i].segments[j].pose.pose.position.x-prev_tracked_humans_.humans[i].segments[j].pose.pose.position.x,
                                          tracked_humans_.humans[i].segments[j].pose.pose.position.y-prev_tracked_humans_.humans[i].segments[j].pose.pose.position.y);

        auto tm_x = tracked_humans_.humans[i].segments[j].pose.pose.position.x;
        auto tm_y = tracked_humans_.humans[i].segments[j].pose.pose.position.y;

        hum_xpos.push_back(tm_x);
        hum_ypos.push_back(tm_y);
        auto n_dist = std::hypot(tm_y - ypos,tm_x - xpos);
        if(tracked_humans_.humans[i].track_id == stuck_human_id)
          ang_theta = std::atan2((tm_y - ypos)/n_dist, (tm_x - xpos)/n_dist);

        if(hum_move_dist<0.0001){
          human_still.push_back(true);
          if(humans_states_.states[i]==hateb_local_planner::HumanState::MOVING){
            humans_states_.states[i] = hateb_local_planner::HumanState::STOPPED;
          }
        }
        else{
          human_still.push_back(false);
        }
      }
    }
  }
  prev_tracked_humans_ = tracked_humans_;

  std::vector<std::pair<double,int>> temp_dist_idx;
  visible_human_ids.clear();
  isDistMax = true;
  for(int i=0;i<human_dists.size();i++){
    auto dist = human_dists[i];
    current_human_dist = human_dists[0];
    if(dist<10.0 && humans_behind[i] >= 0.0){
      isDistMax = false;
      temp_dist_idx.push_back(std::make_pair(dist,i+1));
    }
  }

  if(temp_dist_idx.size()>0){
    std::sort(temp_dist_idx.begin(),temp_dist_idx.end());

    if(human_dists[temp_dist_idx[0].second-1]<=2.5){
      isDistunderThreshold = true;
      }
    else{
      isDistunderThreshold = false;
    }
  }

   if(!stuck){
    int n=500;
    if(temp_dist_idx.size()>=5)
      n=100;
    for(int it=0;it<temp_dist_idx.size();it++){
      //Ray Tracing
      double tm_x =tracked_humans_.humans[temp_dist_idx[it].second-1].segments[0].pose.pose.position.x;
      double tm_y =tracked_humans_.humans[temp_dist_idx[it].second-1].segments[0].pose.pose.position.y;
      auto Dx = (tm_x-xpos)/n;
      auto Dy = (tm_y-ypos)/n;

      //Checking using raytracing
      bool cell_collision = false;
      double rob_x = xpos;
      double rob_y = ypos;

      for(int j=0;j<n;j++){
        unsigned int mx;
        unsigned int my;
        if( costmap_->worldToMap(rob_x,rob_y,mx,my)){
          auto cellcost = costmap_->getCost(mx,my);
          if((int)cellcost==254){
            cell_collision = true;
            break;
          }
          rob_x += Dx;
          rob_y += Dy;
        }
      }
      int hum_id = temp_dist_idx[it].second;

			if(!cell_collision)
			{
        visible_human_ids.push_back(hum_id);
				if(humans_states_.states[hum_id-1]==hateb_local_planner::HumanState::NO_STATE){
					humans_states_.states[hum_id-1] = hateb_local_planner::HumanState::STATIC;
				}
			}
    }
  }
  else{
    for(int it=0;it<2 && it<temp_dist_idx.size();it++){
      if(temp_dist_idx[it].second == stuck_human_id){
        visible_human_ids.push_back(temp_dist_idx[it].second);
        break;
      }
    }
  }

    // Safety step for humans if human_layers is not added in local costmap
    // Adds a temporary costmap around the humans to let planner plan safe trajectories
    auto human_radius = 0.3;
    // if(isMode>=1)
    //   human_radius = 0.08;

    for(int i=0;i<visible_human_ids.size() && i<hum_xpos.size();i++){
    geometry_msgs::Point v1,v2,v3,v4;
    auto idx = visible_human_ids[i]-1;
    v1.x = hum_xpos[idx]-human_radius,v1.y=hum_ypos[idx]-human_radius,v1.z=0.0;
    v2.x = hum_xpos[idx]-human_radius,v2.y=hum_ypos[idx]+human_radius,v2.z=0.0;
    v3.x = hum_xpos[idx]+human_radius,v3.y=hum_ypos[idx]+human_radius,v3.z=0.0;
    v4.x = hum_xpos[idx]+human_radius,v4.y=hum_ypos[idx]-human_radius,v4.z=0.0;

    std::vector<geometry_msgs::Point> human_pos_costmap;

    if(cfg_.robot.is_real){
  		transformStamped = tf_->lookupTransform("odom_combined","map",ros::Time(0),ros::Duration(0.5));
  		tf2::doTransform(v1,v1,transformStamped);
  		tf2::doTransform(v2,v2,transformStamped);
  		tf2::doTransform(v3,v3,transformStamped);
  		tf2::doTransform(v4,v4,transformStamped);
    }

    human_pos_costmap.push_back(v1);
    human_pos_costmap.push_back(v2);
    human_pos_costmap.push_back(v3);
    human_pos_costmap.push_back(v4);

    // if(!human_prev_pos_costmap.empty()){
    //   costmap_->setConvexPolygonCost(human_prev_pos_costmap[idx+1], 0.0);
    // }
    // human_prev_pos_costmap[idx+1] = human_pos_costmap;

    bool set_success = false;
    set_success = costmap_->setConvexPolygonCost(human_pos_costmap, 255.0);
  }
}

bool HATebLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{

  std::string dummy_message;
  geometry_msgs::PoseStamped dummy_pose;
  geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
  uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
  cmd_vel = cmd_vel_stamped.twist;
  return outcome == mbf_msgs::ExePathResult::SUCCESS;
}


uint32_t HATebLocalPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                     const geometry_msgs::TwistStamped& velocity,
                                                     geometry_msgs::TwistStamped &cmd_vel, std::string &message)
{
  auto start_time = ros::Time::now();
  if ((start_time - last_call_time_).toSec() >
      cfg_.hateb.pose_prediction_reset_time) {
    resetHumansPrediction();
  }
  last_call_time_ = start_time;

  // check if plugin initialized
  logs.clear();
  if(!initialized_)
  {
    ROS_ERROR("hateb_local_planner has not been initialized, please call initialize() before using this planner");
    message = "hateb_local_planner has not been initialized";
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }

  if(reset_states){
    for(int i=0;i<humans_states_.states.size();i++){
      humans_states_.states[i]=hateb_local_planner::HumanState::NO_STATE;
      //humans_states_.states[i]=hateb_local_planner::HumanState::STATIC;
    }
    reset_states=false;
  }
  static uint32_t seq = 0;
  cmd_vel.header.seq = seq++;
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  goal_reached_ = false;

  // Get robot pose
  auto pose_get_start_time = ros::Time::now();
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  robot_pose_ = PoseSE2(robot_pose.pose);
  robot_pose_.toPoseMsg(robot_pos_msg);
  // robot_pose_pub_.publish(robot_pos_msg);
  if(std::hypot(robot_pose.pose.position.x-last_robot_pose.position.x, robot_pose.pose.position.y-last_robot_pose.position.y)>0.06){
    last_position_time = ros::Time::now();
  }

  last_robot_pose  = robot_pose.pose;

  if((ros::Time::now()-last_position_time).toSec()>=2.0){
    if(visible_human_ids.size()>0){
      if(human_still[visible_human_ids[0]-1] && isDistunderThreshold && !stuck){
        if(change_mode==0){
          ROS_INFO("I am stuck because of human, Changing to VelObs mode");
        }
        change_mode++;
        isMode = 1;

        if(change_mode>20){
          if(!stuck)
            ROS_INFO("I am stuck");
          stuck = true;
          humans_states_.states[visible_human_ids[0]-1] = hateb_local_planner::HumanState::BLOCKED;
          stuck_human_id = visible_human_ids[0];
          isMode = 2;
        }
      }
    }
  }

  if(!stuck)
    stuck_human_id=-1;

  auto pose_get_time = ros::Time::now() - pose_get_start_time;

  // Get robot velocity
  auto vel_get_start_time = ros::Time::now();
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_.linear.x = robot_vel_tf.pose.position.x;
  robot_vel_.linear.y = robot_vel_tf.pose.position.y;
  robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);
  auto vel_get_time = ros::Time::now() - vel_get_start_time;
  logs+="Velocity: x= " + std::to_string(robot_vel_.linear.x) +", " + " y= " + std::to_string(robot_vel_.linear.y)+", ";
  logs+="dist " + std::to_string(current_human_dist)+", ";

  // prune global plan to cut off parts of the past (spatially before the robot)
  auto prune_start_time = ros::Time::now();
  pruneGlobalPlan(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);
  auto prune_time = ros::Time::now() - prune_start_time;

  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  auto transform_start_time = ros::Time::now();
  PlanCombined transformed_plan_combined;
  int goal_idx;
  geometry_msgs::TransformStamped tf_plan_to_global;
  if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist,
                           transformed_plan_combined, &goal_idx, &tf_plan_to_global))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    message = "Could not transform the global plan to the frame of the controller";
    return mbf_msgs::ExePathResult::INTERNAL_ERROR;
  }
  auto &transformed_plan = transformed_plan_combined.plan_to_optimize;
  auto transform_time = ros::Time::now() - transform_start_time;

  // check if we should enter any backup mode and apply settings
  configureBackupModes(transformed_plan, goal_idx);

  // update via-points container
  // if (!custom_via_points_active_)
  //   updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);

  auto other_start_time = ros::Time::now();
  // check if global goal is reached
  geometry_msgs::PoseStamped global_goal;
  tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
  double dx = global_goal.pose.position.x - robot_pose_.x();
  double dy = global_goal.pose.position.y - robot_pose_.y();
  double delta_orient = g2o::normalize_theta( tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta() );
  if(fabs(std::sqrt(dx*dx+dy*dy)) < cfg_.goal_tolerance.xy_goal_tolerance
    && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance
    && (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0) && goal_ctrl)
  {
    goal_reached_ = true;
    return mbf_msgs::ExePathResult::SUCCESS;
  }

  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
    message = "Transformed plan is empty";
    return mbf_msgs::ExePathResult::INVALID_PATH;
  }

  // Get current goal point (last point of the transformed plan)
  robot_goal_.x() = transformed_plan.back().pose.position.x;
  robot_goal_.y() = transformed_plan.back().pose.position.y;
  // Overwrite goal orientation if needed
  if (cfg_.trajectory.global_plan_overwrite_orientation)
  {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    tf2::convert(q, transformed_plan.back().pose.orientation);
  }
  else
  {
    robot_goal_.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
  }

  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // plan only contains the goal
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
  }
  transformed_plan.front() = robot_pose; // update start

  // clear currently existing obstacles
  obstacles_.clear();
  auto other_time = ros::Time::now() - other_start_time;

  // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
  auto cc_start_time = ros::Time::now();
  if (costmap_converter_)
    updateObstacleContainerWithCostmapConverter();
  else
    updateObstacleContainerWithCostmap();

  // also consider custom obstacles (must be called after other updates, since the container is not cleared)
  updateObstacleContainerWithCustomObstacles();
  auto cc_time = ros::Time::now() - cc_start_time;

  // Do not allow config changes during the following optimization step
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

  // update humans
  auto human_start_time = ros::Time::now();
  std::vector<HumanPlanCombined> transformed_human_plans;
  HumanPlanVelMap transformed_human_plan_vel_map;
  humans_states_pub_.publish(humans_states_);

  switch (cfg_.planning_mode) {
  case 0:
    goal_ctrl = true;
    break;
  case 1: {
    bool found=true;

    if(stuck_human_id!=-1){
      if(visible_human_ids.size()>0){
        if(visible_human_ids[0]!=stuck_human_id)
          found = false;
      }
      else if(visible_human_ids.size()==0)
        found = false;

      // Check for timeout
      if(backed_off && backoff_recovery_.timeOut())
        found = false;
    }

    if(backoff_recovery_.checkRandomRot()){
      break;
    }

    if(isDistMax || !found){
      humans_via_points_map_.clear();

      if(visible_human_ids.size() > 0){
      updateHumanViaPointsContainers(transformed_human_plan_vel_map,
                                     cfg_.trajectory.global_plan_viapoint_sep);
      }
        if(backed_off){
          if (backoff_recovery_.setbackGoal()){
            isMode = 0;
            change_mode = 0;
            backed_off = false;
            stuck = false;
            // goal_ctrl = true;
            if(humans_states_.states[visible_human_ids[0]-1] != hateb_local_planner::HumanState::MOVING)
	             humans_states_.states[visible_human_ids[0]-1] = hateb_local_planner::HumanState::STATIC;
          }
        }
        else
          goal_ctrl = true;

      break;
    }

    if(backoff_recovery_.checkNewGoal()){
      goal_ctrl = true;
      isMode = 0;
      change_mode = 0;
      backed_off = false;
      stuck = false;
      if(humans_states_.states[visible_human_ids[0]-1] != hateb_local_planner::HumanState::MOVING)
         humans_states_.states[visible_human_ids[0]-1] = hateb_local_planner::HumanState::STATIC;
    }

    human_path_prediction::HumanPosePredict predict_srv;

    if(isMode==0)
      isMode = -1;

    for(int i=0;i<2 && i<visible_human_ids.size();i++){
      if((int)humans_states_.states[visible_human_ids[i]-1]>0){
        predict_srv.request.ids.push_back(visible_human_ids[i]);
        if(isMode==-1)
          isMode = 0;
      }
    }

    if (cfg_.hateb.use_external_prediction && change_mode<1) {

      std_srvs::Trigger g_srv;
      human_goal_client_.call(g_srv);
      if(g_srv.response.success && !ext_goal)
        ext_goal = true;

      if (cfg_.hateb.predict_human_behind_robot && !ext_goal) {
        predict_srv.request.type =
            human_path_prediction::HumanPosePredictRequest::BEHIND_ROBOT;

      }
      else if(cfg_.hateb.predict_human_goal && !ext_goal) {
        predict_srv.request.type =
            human_path_prediction::HumanPosePredictRequest::PREDICTED_GOAL;
      }
      else {
        predict_srv.request.type =
            human_path_prediction::HumanPosePredictRequest::EXTERNAL;
      }
    }
    else {
      if(!stuck){
        isMode =1;
      }
      double traj_size = 10, predict_time = 5.0; // TODO: make these values configurable
      for (double i = 1.0; i <= traj_size; ++i) {
        predict_srv.request.predict_times.push_back(predict_time *
                                                    (i / traj_size));
      }
      predict_srv.request.type =
          human_path_prediction::HumanPosePredictRequest::VELOCITY_OBSTACLE;
      if(!backed_off && stuck){
        backed_off = backoff_recovery_.recovery(ang_theta);
        goal_ctrl = false;
      }
    }

    std_srvs::SetBool publish_predicted_markers_srv;
    publish_predicted_markers_srv.request.data =
        publish_predicted_human_markers_;
    if (!publish_predicted_markers_client_ &&
        publish_predicted_markers_client_.call(publish_predicted_markers_srv)) {
      ROS_WARN("Failed to call %s service, is human prediction server running?",
               PUBLISH_MARKERS_SRV_NAME);
    }

    if (predict_humans_client_ && predict_humans_client_.call(predict_srv)) {
      tf2::Stamped<tf2::Transform> tf_human_plan_to_global;
      for (auto predicted_humans_poses :
           predict_srv.response.predicted_humans_poses) {
        if(std::find(predict_srv.request.ids.begin(), predict_srv.request.ids.end(),predicted_humans_poses.id) == predict_srv.request.ids.end()){
          continue;
        }

        if(isMode > 1)
          continue;

        // transform human plans
        HumanPlanCombined human_plan_combined;
        auto &transformed_vel = predicted_humans_poses.start_velocity;

        if (!transformHumanPlan(*tf_, robot_pose, *costmap_, global_frame_,
                                predicted_humans_poses.poses,
                                human_plan_combined, transformed_vel,
                                &tf_human_plan_to_global)) {
          ROS_WARN("Could not transform the human %ld plan to the frame of the "
                   "controller",
                   predicted_humans_poses.id);
          continue;
        }

        human_plan_combined.id = predicted_humans_poses.id;
        transformed_human_plans.push_back(human_plan_combined);

        PlanStartVelGoalVel plan_start_vel_goal_vel;
        plan_start_vel_goal_vel.plan = human_plan_combined.plan_to_optimize;
        plan_start_vel_goal_vel.start_vel = transformed_vel.twist;
        plan_start_vel_goal_vel.nominal_vel = std::max(0.3,human_nominal_vels[predicted_humans_poses.id-1]);
        plan_start_vel_goal_vel.isMode = isMode;
        if (human_plan_combined.plan_after.size() > 0) {
          plan_start_vel_goal_vel.goal_vel = transformed_vel.twist;
        }
        transformed_human_plan_vel_map[human_plan_combined.id] =
            plan_start_vel_goal_vel;
      }
    } else {
      ROS_WARN_THROTTLE(
          THROTTLE_RATE,
          "Failed to call %s service, is human prediction server running?",
          PREDICT_SERVICE_NAME);
    }
    updateHumanViaPointsContainers(transformed_human_plan_vel_map,
                                   cfg_.trajectory.global_plan_viapoint_sep);
    break;
  }
  case 2: {
    human_path_prediction::HumanPosePredict predict_srv;
    predict_srv.request.predict_times.push_back(0.0);
    predict_srv.request.predict_times.push_back(5.0);
    predict_srv.request.type =
        human_path_prediction::HumanPosePredictRequest::VELOCITY_OBSTACLE;

    // setup marker publishing
    std_srvs::SetBool publish_predicted_markers_srv;
    publish_predicted_markers_srv.request.data =
        publish_predicted_human_markers_;
    if (!publish_predicted_markers_client_ &&
        publish_predicted_markers_client_.call(publish_predicted_markers_srv)) {
      ROS_WARN("Failed to call %s service, is human prediction server running?",
               PUBLISH_MARKERS_SRV_NAME);
    }

    // call human prediction server
    if (predict_humans_client_ && predict_humans_client_.call(predict_srv)) {
      for (auto predicted_humans_poses :
           predict_srv.response.predicted_humans_poses) {
        if (predicted_humans_poses.id == cfg_.approach.approach_id) {
          geometry_msgs::PoseStamped transformed_human_pose;
          if (!transformHumanPose(*tf_, global_frame_,
                                  predicted_humans_poses.poses.front(),
                                  transformed_human_pose)) {
            ROS_WARN(
                "Could not transform the human %ld pose to controller frame",
                predicted_humans_poses.id);
          }

          PlanStartVelGoalVel plan_start_vel_goal_vel;
          plan_start_vel_goal_vel.plan.push_back(transformed_human_pose);
          plan_start_vel_goal_vel.nominal_vel = std::max(0.3,human_nominal_vels[predicted_humans_poses.id-1]);
          plan_start_vel_goal_vel.isMode = isMode;
          transformed_human_plan_vel_map[predicted_humans_poses.id] =
              plan_start_vel_goal_vel;

          // update global plan of the robot
          // find position in front of the human
          tf2::Transform tf_human_pose, tf_approach_pose[3];
            geometry_msgs::PoseStamped approach_pose[3];
            for (int i=0; i < 3; i++){
                tf2::fromMsg(transformed_human_pose.pose, tf_human_pose);
                tf_approach_pose[i].setOrigin(tf2::Vector3(cfg_.approach.approach_dist + (2 - i) * 0.3, 0.0, 0.0));
		            tf2::Quaternion approachQuaternion;
                approachQuaternion.setEuler(cfg_.approach.approach_angle,0.0,0.0);
                tf_approach_pose[i].setRotation(approachQuaternion);
                tf_approach_pose[i] = tf_human_pose * tf_approach_pose[i];
                tf2::toMsg(tf_approach_pose[i], approach_pose[i].pose);
                approach_pose[i].header = transformed_human_pose.header;
            }

          // add approach pose to the robot plan, only within reachable distance
          // auto &plan_goal = transformed_plan.back().pose;
          // auto &approach_goal = approach_pose[2].pose;
          tf2::Transform plan_goal,approach_goal;
          tf2::fromMsg(transformed_plan.back().pose,plan_goal);
          tf2::fromMsg(approach_pose[2].pose,approach_goal);
          double lin_dist = std::abs(
              std::hypot(plan_goal.getOrigin().getX() - approach_goal.getOrigin().getX(),
                         plan_goal.getOrigin().getY() - approach_goal.getOrigin().getY()));
          double ang_dist = std::abs(angles::shortest_angular_distance(
              tf2::impl::getYaw(plan_goal.getRotation()),
              tf2::impl::getYaw(approach_goal.getRotation())));
          // ROS_INFO("lin_dist=%.2f, ang_dist=%.2f", lin_dist, ang_dist);
            tf2::Transform tf_approach_global[3];
            geometry_msgs::PoseStamped approach_pose_global[3];
          if (lin_dist > cfg_.approach.approach_dist_tolerance ||
              ang_dist > cfg_.approach.approach_angle_tolerance) {
              for (int i = 0; i < 3; i++) {
                  transformed_plan.push_back(approach_pose[i]);

                  // get approach poses in to the frame of global plan
                  tf2::Stamped<tf2::Transform> temp;
                  tf2::fromMsg(tf_plan_to_global,temp);
                  tf_approach_global[i] = temp.inverse() * tf_approach_pose[i];

                  tf2::toMsg(tf_approach_global[i], approach_pose_global[i].pose);
                  approach_pose_global[i].header = global_plan_.back().header;
              }

            // prune and update global plan
            auto global_plan_it = global_plan_.begin();
            double last_dist = std::numeric_limits<double>::infinity();
            while (global_plan_it != global_plan_.end()) {
              auto &p_pos = (*global_plan_it).pose.position;
              auto &a_pos = approach_pose_global[0].pose.position;
              double pa_dist = std::hypot(p_pos.x - a_pos.x, p_pos.y - a_pos.y);
              if (pa_dist > last_dist) {
                break;
              }
              last_dist = pa_dist;
              global_plan_it++;
            }
            global_plan_.erase(global_plan_it, global_plan_.end());
              for (int i = 0; i < 3; i++) {
                  global_plan_.push_back(approach_pose_global[i]);
              }
            ROS_INFO("Global plan modified for approach behavior");
          }
        }
      }
    } else {
      ROS_WARN_THROTTLE(
          THROTTLE_RATE,
          "Failed to call %s service, is human prediction server running?",
          PREDICT_SERVICE_NAME);
    }
    // TODO: check if plan-map is not empty
    break;
  }
  default:
    break;
  }

  std::string mode;
  if(isMode==-1 || isDistMax){
    mode = "SingleBand";
  }
  else if(isMode==0){
    mode = "DualBand";
  }
  else if(isMode == 1){
    mode = "VelObs";
  }
  else if(isMode == 2){
    mode = "Backoff";
  }
  logs+="Mode: " + mode+", ";

  std_msgs::String log_msg;
  log_msg.data = logs;
  log_pub_.publish(log_msg);

  auto human_time = ros::Time::now() - human_start_time;

  // update via-points container
  auto via_start_time = ros::Time::now();
  // overwrite/update start of the transformed plan with the actual robot
  // position (allows using the plan as initial trajectory)
  // tf::poseTFToMsg(robot_pose, transformed_plan.front().pose);
  transformed_plan.front()=robot_pose;
  if (!custom_via_points_active_)
    updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);
  auto via_time = ros::Time::now() - via_start_time;

  // Now perform the actual
  auto plan_start_time = ros::Time::now();
  // bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_.goal_tolerance.free_goal_vel); // straight line init
  hateb_local_planner::OptimizationCostArray op_costs;

  double dt_resize=cfg_.trajectory.dt_ref;
  double dt_hyst_resize=cfg_.trajectory.dt_hysteresis;

  // Uncomment the below lines of code for intermediate band tightness change
  // if(isDistunderThreshold && isMode==0){
  //     dt_resize = 0.2;
  //     dt_hyst_resize = 0.1;
  //   }

  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel, &transformed_human_plan_vel_map, &op_costs, dt_resize, dt_hyst_resize);

  if (!success)
  {
    planner_->clearPlanner(); // force reinitialization for next time
    ROS_WARN("hateb_local_planner was not able to obtain a local plan for the current setting.");

    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "hateb_local_planner was not able to obtain a local plan";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }
  // op_costs_pub_.publish(op_costs);
  auto plan_time = ros::Time::now() - plan_start_time;

  PlanTrajCombined plan_traj_combined;
  plan_traj_combined.plan_before = transformed_plan_combined.plan_before;
  planner_->getFullTrajectory(plan_traj_combined.optimized_trajectory);
  plan_traj_combined.plan_after = transformed_plan_combined.plan_after;
  visualization_->publishTrajectory(plan_traj_combined);

  if (cfg_.planning_mode == 1) {
    visualization_->publishHumanGlobalPlans(transformed_human_plans);
    std::vector<HumanPlanTrajCombined> human_plans_traj_array;
    for (auto &human_plan_combined : transformed_human_plans) {
      HumanPlanTrajCombined human_plan_traj_combined;
      human_plan_traj_combined.id = human_plan_combined.id;
      human_plan_traj_combined.plan_before = human_plan_combined.plan_before;
      planner_->getFullHumanTrajectory(
          human_plan_traj_combined.id,
          human_plan_traj_combined.optimized_trajectory);

      human_plan_traj_combined.plan_after = human_plan_combined.plan_after;
      human_plans_traj_array.push_back(human_plan_traj_combined);

    }
    visualization_->publishHumanTrajectories(human_plans_traj_array);
  }

  double ttg = std::hypot(transformed_plan.back().pose.position.x - transformed_plan.front().pose.position.x,
                transformed_plan.back().pose.position.y - transformed_plan.front().pose.position.y)/std::hypot(robot_vel_.linear.x,robot_vel_.linear.y);

  // Undo temporary horizon reduction
  auto hr2_start_time = ros::Time::now();

  auto hr2_time = ros::Time::now() - hr2_start_time;

  // Check feasibility (but within the first few states only)
  auto fsb_start_time = ros::Time::now();
  if(cfg_.robot.is_footprint_dynamic)
  {
    // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
  }
  bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses);
  if (!feasible)
  {
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
      // now we reset everything to start again with the initialization of new trajectories.
      planner_->clearPlanner();
      ROS_WARN("HATebLocalPlannerROS: trajectory is not feasible. Resetting planner...");

      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = ros::Time::now();
      last_cmd_ = cmd_vel.twist;

      message = "hateb_local_planner trajectory is not feasible";
      return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }
  auto fsb_time = ros::Time::now() - fsb_start_time;

  // Get the velocity command for this sampling interval
  auto vel_start_time = ros::Time::now();
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_.trajectory.control_look_ahead_poses, dt_resize))
  {
    planner_->clearPlanner();
    ROS_WARN("HATebLocalPlannerROS: velocity command invalid. Resetting planner...");
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "hateb_local_planner velocity command invalid";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
		   cfg_.robot.max_vel_x, cfg_.robot.max_vel_y, cfg_.robot.max_vel_theta,
		   cfg_.robot.max_vel_x_backwards);

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
  // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
  if (cfg_.robot.cmd_angle_instead_rotvel)
  {
    cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, cfg_.robot.wheelbase, 0.95*cfg_.robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.twist.angular.z))
    {
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
      last_cmd_ = cmd_vel.twist;
      planner_->clearPlanner();
      ROS_WARN("HATebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...");
      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = ros::Time::now();

      message = "hateb_local_planner steering angle is not finite";
      return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }
  }
  auto vel_time = ros::Time::now() - vel_start_time;

  // a feasible solution should be found, reset counter
  no_infeasible_plans_ = 0;

  // store last command (for recovery analysis etc.)
  last_cmd_ = cmd_vel.twist;

  // Now visualize everything
  auto viz_start_time = ros::Time::now();
  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  if(isDistMax)
    visualization_->publishMode(-1);
  else
    visualization_->publishMode(isMode);

  auto viz_time = ros::Time::now() - viz_start_time;
  auto total_time = ros::Time::now() - start_time;

  ROS_DEBUG_STREAM_COND(total_time.toSec() > 0.1, "\tcompute velocity times:\n"
          << "\t\ttotal time                   "
          << std::to_string(total_time.toSec()) << "\n"
          << "\t\tpose get time                "
          << std::to_string(pose_get_time.toSec()) << "\n"
          << "\t\tvel get time                 "
          << std::to_string(vel_get_time.toSec()) << "\n"
          << "\t\tprune time                   "
          << std::to_string(prune_time.toSec()) << "\n"
          << "\t\ttransform time               "
          << std::to_string(transform_time.toSec()) << "\n"
          // << "\t\thorizon setup time           "
          // << std::to_string((hr1_time + hr2_time).toSec()) << "\n"
          << "\t\tother time                   "
          << std::to_string(other_time.toSec()) << "\n"
          << "\t\tcostmap convert time         "
          << std::to_string(cc_time.toSec()) << "\n"
          << "\t\tvia points time              "
          << std::to_string(via_time.toSec()) << "\n"
          << "\t\thuman time                   "
          << std::to_string(human_time.toSec()) << "\n"
          << "\t\tplanning time                "
          << std::to_string(plan_time.toSec()) << "\n"
          << "\t\tplan feasibility check time  "
          << std::to_string(fsb_time.toSec()) << "\n"
          << "\t\tvelocity extract time        "
          << std::to_string(vel_time.toSec()) << "\n"
          << "\t\tvisualization publish time   "
          << std::to_string(viz_time.toSec()) << "\n=========================");
  return mbf_msgs::ExePathResult::SUCCESS;
}


bool HATebLocalPlannerROS::isGoalReached()
{
  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    planner_->clearPlanner();
    resetHumansPrediction();
    change_mode = 0;
    isMode = 0;
    ext_goal = false;
    stuck = false;
    goal_ctrl = true;
    human_still.clear();
    humans_states_.states.clear();
    // states_.states.clear();
    isDistunderThreshold = false;
    backed_off =  false;
    reset_states=true;
    stuck_human_id = -1;
    return true;
  }
  return false;
}



void HATebLocalPlannerROS::updateObstacleContainerWithCostmap()
{
  // Add costmap obstacles if desired
  if (cfg_.obstacles.include_costmap_obstacles)
  {
    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

    for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
    {
      for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
      {
        if (costmap_->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE)
        {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));

          // check if obstacle is interesting (e.g. not far behind the robot)
          Eigen::Vector2d obs_dir = obs-robot_pose_.position();
          if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist  )
            continue;

          obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
        }
      }
    }
  }
}

void HATebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
{
  if (!costmap_converter_)
    return;

  //Get obstacles from costmap converter
  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  if (!obstacles)
    return;

  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::Polygon* polygon = &obstacle->polygon;

    if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
    {
      obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
    }
    else if (polygon->points.size()==1) // Point
    {
      obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
    }
    else if (polygon->points.size()==2) // Line
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        polygon->points[1].x, polygon->points[1].y )));
    }
    else if (polygon->points.size()>2) // Real polygon
    {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (std::size_t j=0; j<polygon->points.size(); ++j)
        {
            polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
    }

    // Set velocity, if obstacle is moving
    if(!obstacles_.empty())
      obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
  }
}


void HATebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
{
  // Add custom obstacles obtained via message
  boost::mutex::scoped_lock l(custom_obst_mutex_);

  if (!custom_obstacle_msg_.obstacles.empty())
  {
    // We only use the global header to specify the obstacle coordinate system instead of individual ones
    Eigen::Affine3d obstacle_to_map_eig;
    try
    {
      geometry_msgs::TransformStamped obstacle_to_map =  tf_->lookupTransform(global_frame_, ros::Time::now(),
                                                                              custom_obstacle_msg_.header.frame_id, ros::Time::now(),
                                                                              custom_obstacle_msg_.header.frame_id, ros::Duration(0.5));
      obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      obstacle_to_map_eig.setIdentity();
    }

    for (size_t i=0; i<custom_obstacle_msg_.obstacles.size(); ++i)
    {
      if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0 ) // circle
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new CircularObstacle( (obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 ) // point
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new PointObstacle( (obstacle_to_map_eig * pos).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2 ) // line
      {
        Eigen::Vector3d line_start( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        Eigen::Vector3d line_end( custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z );
        obstacles_.push_back(ObstaclePtr(new LineObstacle( (obstacle_to_map_eig * line_start).head(2),
                                                           (obstacle_to_map_eig * line_end).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
      {
        ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
        continue;
      }
      else // polygon
      {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (size_t j=0; j<custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
        {
          Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z );
          polyobst->pushBackVertex( (obstacle_to_map_eig * pos).head(2) );
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
      }

      // Set velocity, if obstacle is moving
      if(!obstacles_.empty())
        obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
    }
  }
}

void HATebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation)
{
  via_points_.clear();

  if (min_separation<=0)
    return;

  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
      continue;

    // add via-point
    via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
    prev_idx = i;
  }
}


Eigen::Vector2d HATebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
{
  Eigen::Vector2d vel;
  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
  return vel;
}

void HATebLocalPlannerROS::updateHumanViaPointsContainers(
    const HumanPlanVelMap &transformed_human_plan_vel_map,
    double min_separation) {
  if (min_separation < 0)
    return;

  // reset via-points for known humans, create via-points for new humans
  for (auto &transformed_human_plan_vel_kv : transformed_human_plan_vel_map)
  {
    auto &human_id = transformed_human_plan_vel_kv.first;
    if (humans_via_points_map_.find(human_id) != humans_via_points_map_.end())
      {
        humans_via_points_map_[human_id].clear();
      }
    else
      {
        humans_via_points_map_[human_id] = ViaPointContainer();
      }
  }

  // remove human via-points for vanished humans
  auto itr = humans_via_points_map_.begin();
  while (itr != humans_via_points_map_.end())
  {
    if (transformed_human_plan_vel_map.count(itr->first) == 0){
      itr = humans_via_points_map_.erase(itr);
    }
    else
      ++itr;
  }

  std::size_t prev_idx;
  for (auto &transformed_human_plan_vel_kv : transformed_human_plan_vel_map) {
    prev_idx = 0;
    auto &human_id = transformed_human_plan_vel_kv.first;
    auto &transformed_human_plan = transformed_human_plan_vel_kv.second.plan;
    for (std::size_t i = 1; i < transformed_human_plan.size(); ++i) {
      if (distance_points2d(transformed_human_plan[prev_idx].pose.position, transformed_human_plan[i].pose.position) < min_separation)
        continue;
      humans_via_points_map_[human_id].push_back(Eigen::Vector2d(transformed_human_plan[i].pose.position.x, transformed_human_plan[i].pose.position.y));
      prev_idx = i;
    }
  }
}

bool HATebLocalPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return true;

  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
    geometry_msgs::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);

    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;

    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
         erase_end = it;
         break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;

    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}

bool HATebLocalPlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                  PlanCombined &transformed_plan_combined, int* current_goal_idx, geometry_msgs::TransformStamped* tf_plan_to_global) const

{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan_combined.plan_to_optimize.clear();

  try
  {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(0), plan_pose.header.frame_id, plan_pose.header.stamp,
                                                                                  plan_pose.header.frame_id, ros::Duration(0.5));

    //let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id,ros::Duration(0.05));

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0)*2.0;
    // dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                           // located on the border of the local costmap
    dist_threshold *= 0.9;


    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;

    tf2::Stamped<tf2::Transform> tf_pose;
    geometry_msgs::PoseStamped newer_pose;
    //we need to loop to a point on the plan that is within a certain distance of the robot
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (new_sq_dist > sq_dist_threshold)
        break;  // force stop if we have reached the costmap border

      if (new_sq_dist < sq_dist) // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
      }

      const geometry_msgs::PoseStamped &pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan_combined.plan_before.push_back(newer_pose);
    }
    double plan_length = 0; // check cumulative Euclidean distance along the plan

    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan_combined.plan_to_optimize.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;

      // caclulate distance to previous pose
      if (i>0 && max_plan_length>0)
        plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);
      ++i;
    }

    // // // Modification for hateb_local_planner:
    // // // Return the index of the current goal point (inside the distance
    // // // threshold)
    if (current_goal_idx)
      *current_goal_idx = i - 1; // minus 1, since i was increased once before leaving the loop

    while (i < global_plan.size()) {
      const geometry_msgs::PoseStamped &pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);
      transformed_plan_combined.plan_after.push_back(newer_pose);
      ++i;
    }

    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan_combined.plan_after.empty())
    {
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan_combined.plan_after.push_back(newer_pose);

      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }

    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex)
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex)
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }

  return true;
}


int HATebLocalPlannerROS::getLatestCommonTime(const std::string &source_frame, const std::string &target_frame, ros::Time& time, std::string* error_string) const
{
  tf2::CompactFrameID target_id = tf_->_lookupFrameNumber(tf::strip_leading_slash(target_frame));
  tf2::CompactFrameID source_id = tf_->_lookupFrameNumber(tf::strip_leading_slash(source_frame));

  return tf_->_getLatestCommonTime(source_id, target_id, time, error_string);
}

void HATebLocalPlannerROS::lookupTwist(const std::string& tracking_frame, const std::string& observation_frame,
                              const ros::Time& time, const ros::Duration& averaging_interval,
                              geometry_msgs::Twist& twist) const
{
  // ref point is origin of tracking_frame, ref_frame = obs_frame
  lookupTwist(tracking_frame, observation_frame, observation_frame, tf2::Vector3(0,0,0), tracking_frame, time, averaging_interval, twist);
};

void HATebLocalPlannerROS::lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const std::string& reference_frame,
                 const tf2::Vector3 & reference_point, const std::string& reference_point_frame,
                 const ros::Time& time, const ros::Duration& averaging_interval,
                 geometry_msgs::Twist& twist) const
{

  ros::Time latest_time, target_time;
  getLatestCommonTime(observation_frame, tracking_frame, latest_time, NULL); ///\TODO check time on reference point too

  if (ros::Time() == time)
    target_time = latest_time;
  else
    target_time = time;

  ros::Time end_time = std::min(target_time + averaging_interval *0.5 , latest_time);

  ros::Time start_time = std::max(ros::Time().fromSec(.00001) + averaging_interval, end_time) - averaging_interval;  // don't collide with zero
  ros::Duration corrected_averaging_interval = end_time - start_time; //correct for the possiblity that start time was truncated above.
  geometry_msgs::TransformStamped start_msg, end_msg;
  start_msg = tf_->lookupTransform(observation_frame, tracking_frame, start_time);
  end_msg = tf_->lookupTransform(observation_frame, tracking_frame, end_time);

  tf2::Stamped< tf2::Transform > start,end;
  tf2::fromMsg(start_msg,start);
  tf2::fromMsg(end_msg,end);

  tf2::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
  tf2::Quaternion quat_temp;
  temp.getRotation(quat_temp);
  tf2::Vector3 o = start.getBasis() * quat_temp.getAxis();
  tfScalar ang = quat_temp.getAngle();

  double delta_x = end.getOrigin().getX() - start.getOrigin().getX();
  double delta_y = end.getOrigin().getY() - start.getOrigin().getY();
  double delta_z = end.getOrigin().getZ() - start.getOrigin().getZ();


  tf2::Vector3 twist_vel ((delta_x)/corrected_averaging_interval.toSec(),
                       (delta_y)/corrected_averaging_interval.toSec(),
                       (delta_z)/corrected_averaging_interval.toSec());
  tf2::Vector3 twist_rot = o * (ang / corrected_averaging_interval.toSec());


  // This is a twist w/ reference frame in observation_frame  and reference point is in the tracking_frame at the origin (at start_time)


  //correct for the position of the reference frame
  tf2::Stamped< tf2::Transform > inverse;
  tf2::fromMsg(tf_->lookupTransform(reference_frame,tracking_frame,  target_time),inverse);
  tf2::Vector3 out_rot = inverse.getBasis() * twist_rot;
  tf2::Vector3 out_vel = inverse.getBasis()* twist_vel + inverse.getOrigin().cross(out_rot);


  //Rereference the twist about a new reference point
  // Start by computing the original reference point in the reference frame:
  tf2::Stamped<tf2::Vector3> rp_orig(tf2::Vector3(0,0,0), target_time, tracking_frame);
  geometry_msgs::TransformStamped reference_frame_trans;
  tf2::fromMsg(tf_->lookupTransform(reference_frame,rp_orig.frame_id_,rp_orig.stamp_),reference_frame_trans);

  geometry_msgs::PointStamped rp_orig_msg;
  tf2::toMsg(rp_orig,rp_orig_msg);
  tf2::doTransform(rp_orig_msg, rp_orig_msg, reference_frame_trans);

  // convert the requrested reference point into the right frame
  tf2::Stamped<tf2::Vector3> rp_desired(reference_point, target_time, reference_point_frame);
  geometry_msgs::PointStamped rp_desired_msg;
  tf2::toMsg(rp_desired,rp_desired_msg);
  tf2::doTransform(rp_desired_msg, rp_desired_msg, reference_frame_trans);
  // compute the delta
  tf2::Vector3 delta = rp_desired - rp_orig;
  // Correct for the change in reference point
  out_vel = out_vel + out_rot * delta;
  // out_rot unchanged

  /*
    printf("KDL: Rotation %f %f %f, Translation:%f %f %f\n",
         out_rot.x(),out_rot.y(),out_rot.z(),
         out_vel.x(),out_vel.y(),out_vel.z());
  */

  twist.linear.x =  out_vel.x();
  twist.linear.y =  out_vel.y();
  twist.linear.z =  out_vel.z();
  twist.angular.x =  out_rot.x();
  twist.angular.y =  out_rot.y();
  twist.angular.z =  out_rot.z();

};

bool HATebLocalPlannerROS::transformHumanPlan(
    const tf2_ros::Buffer &tf2, const geometry_msgs::PoseStamped &robot_pose,
    const costmap_2d::Costmap2D &costmap, const std::string &global_frame,
    const std::vector<geometry_msgs::PoseWithCovarianceStamped> &human_plan,
    HumanPlanCombined &transformed_human_plan_combined,
    geometry_msgs::TwistStamped &transformed_human_twist,
    tf2::Stamped<tf2::Transform> *tf_human_plan_to_global) const {
  try {
    if (human_plan.empty()) {
      ROS_ERROR("Received human plan with zero length");
      return false;
    }

    // get human_plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped human_plan_to_global_transform;
    // tf.waitForTransform(global_frame, human_plan.front().header.frame_id,
                        // ros::Time(0), ros::Duration(0.5));
    human_plan_to_global_transform = tf2.lookupTransform(global_frame, human_plan.front().header.frame_id,
                                                                        ros::Time(0),ros::Duration(0.5));
    tf2::Stamped< tf2::Transform > human_plan_to_global_transform_;
    tf2::fromMsg(human_plan_to_global_transform,human_plan_to_global_transform_);

    // transform the full plan to local planning frame
    std::vector<geometry_msgs::PoseStamped> transformed_human_plan;
    tf2::Stamped<tf2::Transform> tf_pose_stamped;
    geometry_msgs::PoseStamped transformed_pose;
    tf2::Transform tf_pose;
    auto agent_start_pose = agent_plan[0];
    for (auto &human_pose : human_plan) {
      if(isMode>=1){
        if(std::hypot(agent_pose.pose.pose.position.x - agent_start_pose.pose.pose.position.x,
                      agent_pose.pose.pose.position.y - agent_start_pose.pose.pose.position.y) > (cfg_.agent.radius)){
          unsigned int mx, my;
          if(costmap_->worldToMap(human_pose.pose.pose.position.x,human_pose.pose.pose.position.y,mx,my)){
            if(costmap_->getCost(mx,my)>=254)
              break;
          }
        }
      }
      tf2::fromMsg(human_pose.pose.pose, tf_pose);
      tf_pose_stamped.setData(human_plan_to_global_transform_ * tf_pose);
      tf_pose_stamped.stamp_ = human_plan_to_global_transform_.stamp_;
      tf_pose_stamped.frame_id_ = global_frame;
      tf2::toMsg(tf_pose_stamped, transformed_pose);

      transformed_human_plan.push_back(transformed_pose);
    }

    // transform human twist to local planning frame
    geometry_msgs::Twist human_to_global_twist;
    lookupTwist(global_frame, transformed_human_twist.header.frame_id,
                   ros::Time(0), ros::Duration(0.5), human_to_global_twist);
    transformed_human_twist.twist.linear.x -= human_to_global_twist.linear.x;
    transformed_human_twist.twist.linear.y -= human_to_global_twist.linear.y;
    transformed_human_twist.twist.angular.z -= human_to_global_twist.angular.z;

    double dist_threshold =
        std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                 costmap.getSizeInCellsY() * costmap.getResolution() / 2.0) *
        2.0;
    dist_threshold *= 0.9;

    double sq_dist_threshold = dist_threshold * dist_threshold;
    double x_diff, y_diff, sq_dist;

    // get first point of human plan within threshold distance from robot
    int start_index = transformed_human_plan.size(), end_index = 0;
    for (int i = 0; i < transformed_human_plan.size(); i++) {
      x_diff = robot_pose.pose.position.x -
               transformed_human_plan[i].pose.position.x;
      y_diff = robot_pose.pose.position.y -
               transformed_human_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (sq_dist < sq_dist_threshold) {
        start_index = i;
        break;
      }
    }
    // now get last point of human plan withing threshold distance from robot
    for (int i = (transformed_human_plan.size() - 1); i >= 0; i--) {
      x_diff = robot_pose.pose.position.x -
               transformed_human_plan[i].pose.position.x;
      y_diff = robot_pose.pose.position.y -
               transformed_human_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (sq_dist < sq_dist_threshold) {
        end_index = i;
        break;
      }
    }

    // ROS_INFO("start: %d, end: %d, full: %ld", start_index, end_index,
    // transformed_human_plan.size());
    transformed_human_plan_combined.plan_before.clear();
    transformed_human_plan_combined.plan_to_optimize.clear();
    transformed_human_plan_combined.plan_after.clear();
    for (int i = 0; i < transformed_human_plan.size(); i++) {
      if (i < start_index) {
        transformed_human_plan_combined.plan_before.push_back(
            transformed_human_plan[i]);
      } else if (i >= start_index && i <= end_index) {
        transformed_human_plan_combined.plan_to_optimize.push_back(
            transformed_human_plan[i]);
      } else if (i > end_index) {
        transformed_human_plan_combined.plan_after.push_back(
            transformed_human_plan[i]);
      } else {
        ROS_ERROR("Transform human plan indexing error");
      }
    }

    if (tf_human_plan_to_global)
      *tf_human_plan_to_global = human_plan_to_global_transform_;
  } catch (tf::LookupException &ex) {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  } catch (tf::ExtrapolationException &ex) {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (human_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n",
                global_frame.c_str(), (unsigned int)human_plan.size(),
                human_plan.front().header.frame_id.c_str());

    return false;
  }

  return true;
}

bool HATebLocalPlannerROS::transformHumanPose(
    const tf2_ros::Buffer &tf2, const std::string &global_frame,
    geometry_msgs::PoseWithCovarianceStamped &human_pose,
    geometry_msgs::PoseStamped &transformed_human_pose) const {
  try {
    // get human_pose_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped human_plan_to_global_transform;
    human_plan_to_global_transform = tf2.lookupTransform(global_frame, human_pose.header.frame_id, ros::Time(0),
                       ros::Duration(0.5));
    tf2::Stamped< tf2::Transform > human_plan_to_global_transform_;
    tf2::fromMsg(human_plan_to_global_transform,human_plan_to_global_transform_);

    // transform human pose to local planning frame
    tf2::Stamped<tf2::Transform> tf_pose_stamped;
    tf2::Transform tf_pose;
    tf2::fromMsg(human_pose.pose.pose, tf_pose);
    tf_pose_stamped.setData(human_plan_to_global_transform_ * tf_pose);
    tf_pose_stamped.stamp_ = human_plan_to_global_transform_.stamp_;
    tf_pose_stamped.frame_id_ = global_frame;
    tf2::toMsg(tf_pose_stamped, transformed_human_pose);
  } catch (tf::LookupException &ex) {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  } catch (tf::ExtrapolationException &ex) {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    ROS_ERROR("Global Frame: %s Pose Frame %s\n", global_frame.c_str(),
              human_pose.header.frame_id.c_str());
    return false;
  }

  return true;
}

double HATebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
              int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
  int n = (int)global_plan.size();

  // check if we are near the global goal already
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // we've exactly reached the goal
    {
      return tf2::getYaw(local_goal.pose.orientation);
    }
    else
    {
      tf2::Quaternion global_orientation;
      tf2::convert(global_plan.back().pose.orientation, global_orientation);
      tf2::Quaternion rotation;
      tf2::convert(tf_plan_to_global.transform.rotation, rotation);
      // TODO(roesmann): avoid conversion to tf2::Quaternion
      return tf2::getYaw(rotation *  global_orientation);
    }
  }

  // reduce number of poses taken into account if the desired number of poses is not available
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before

  std::vector<double> candidates;
  geometry_msgs::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::PoseStamped tf_pose_kp1;

  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // Transform pose of the global plan to the planning frame
    tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);

    // calculate yaw angle
    candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
        tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );

    if (i<range_end-1)
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);
}


void HATebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards)
{
  // Limit translational velocity for forward driving
  if (vx > max_vel_x)
    vx = max_vel_x;

  // limit strafing velocity
  if (vy > max_vel_y)
    vy = max_vel_y;
  else if (vy < -max_vel_y)
    vy = -max_vel_y;

  // Limit angular velocity
  if (omega > max_vel_theta)
    omega = max_vel_theta;
  else if (omega < -max_vel_theta)
    omega = -max_vel_theta;

  // Limit backwards velocity
  if (max_vel_x_backwards<=0)
  {
    ROS_WARN_ONCE("HATebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  }
  else if (vx < -max_vel_x_backwards)
    vx = -max_vel_x_backwards;

  // slow change of direction in angular velocity
  double min_vel_theta = 0.02;
  if (cfg_.optim.disable_rapid_omega_chage) {
    if (std::signbit(omega) != std::signbit(last_omega_)) {
      // signs are changed
      auto now = ros::Time::now();
      if ((now - last_omega_sign_change_).toSec() <
          cfg_.optim.omega_chage_time_seperation) {
        // do not allow sign change
        omega = std::copysign(min_vel_theta, omega);
      }
      last_omega_sign_change_ = now;
      last_omega_ = omega;
    }
  }
}

double HATebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
{
  if (omega==0 || v==0)
    return 0;

  double radius = v/omega;

  if (fabs(radius) < min_turning_radius)
    radius = double(g2o::sign(radius)) * min_turning_radius;

  return std::atan(wheelbase / radius);
}

void HATebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    ROS_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}



void HATebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan,  int& goal_idx)
{
    ros::Time current_time = ros::Time::now();

    // reduced horizon backup mode
    if (cfg_.recovery.shrink_horizon_backup &&
        goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
       (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).toSec() < cfg_.recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
    {
        ROS_INFO_COND(no_infeasible_plans_==1, "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_.recovery.shrink_horizon_min_duration);


        // Shorten horizon if requested
        // reduce to 50 percent:
        int horizon_reduction = goal_idx/2;

        if (no_infeasible_plans_ > 9)
        {
            ROS_INFO_COND(no_infeasible_plans_==10, "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
            horizon_reduction /= 2;
        }

        // we have a small overhead here, since we already transformed 50% more of the trajectory.
        // But that's ok for now, since we do not need to make transformGlobalPlan more complex
        // and a reduced horizon should occur just rarely.
        int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
        goal_idx -= horizon_reduction;
        if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
            transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
        else
            goal_idx += horizon_reduction; // this should not happen, but safety first ;-)
    }


    // detect and resolve oscillations
    if (cfg_.recovery.oscillation_recovery)
    {
        double max_vel_theta;
        double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_.robot.max_vel_x : cfg_.robot.max_vel_x_backwards;
        if (cfg_.robot.min_turning_radius!=0 && max_vel_current>0)
            max_vel_theta = std::max( max_vel_current/std::abs(cfg_.robot.min_turning_radius),  cfg_.robot.max_vel_theta );
        else
            max_vel_theta = cfg_.robot.max_vel_theta;

        failure_detector_.update(last_cmd_, cfg_.robot.max_vel_x, cfg_.robot.max_vel_x_backwards, max_vel_theta,
                               cfg_.recovery.oscillation_v_eps, cfg_.recovery.oscillation_omega_eps);

        bool oscillating = failure_detector_.isOscillating();
        bool recently_oscillated = (ros::Time::now()-time_last_oscillation_).toSec() < cfg_.recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently

        if (oscillating)
        {
            if (!recently_oscillated)
            {
                // save current turning direction
                if (robot_vel_.angular.z > 0)
                    last_preferred_rotdir_ = RotType::left;
                else
                    last_preferred_rotdir_ = RotType::right;
                ROS_WARN("HATebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
            }
            time_last_oscillation_ = ros::Time::now();
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
        }
        else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
        {
            last_preferred_rotdir_ = RotType::none;
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
            ROS_INFO("HATebLocalPlannerROS: oscillation recovery disabled/expired.");
        }
    }

}


void HATebLocalPlannerROS::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  boost::mutex::scoped_lock l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;
}

void HATebLocalPlannerROS::customViaPointsCB(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  if (cfg_.trajectory.global_plan_viapoint_sep > 0)
  {
    ROS_WARN("Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
             "Ignoring custom via-points.");
    custom_via_points_active_ = false;
    return;
  }

  boost::mutex::scoped_lock l(via_point_mutex_);
  via_points_.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  custom_via_points_active_ = !via_points_.empty();
}

RobotFootprintModelPtr HATebLocalPlannerROS::getRobotFootprintFromParamServer(const ros::NodeHandle& nh)
{
  std::string model_name;
  if (!nh.getParam("footprint_model/type", model_name))
  {
    ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
    return boost::make_shared<PointRobotFootprint>();
  }

  // point
  if (model_name.compare("point") == 0)
  {
    ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
    return boost::make_shared<PointRobotFootprint>();
  }

  // circular
  if (model_name.compare("circular") == 0)
  {
    // get radius
    double radius;
    if (!nh.getParam("footprint_model/radius", radius))
    {
      ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                       << "/footprint_model/radius' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius <<"m) loaded for trajectory optimization.");
    return boost::make_shared<CircularRobotFootprint>(radius);
  }

  // line
  if (model_name.compare("line") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end"))
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                       << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get line coordinates
    std::vector<double> line_start, line_end;
    nh.getParam("footprint_model/line_start", line_start);
    nh.getParam("footprint_model/line_end", line_end);
    if (line_start.size() != 2 || line_end.size() != 2)
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                       << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }

    ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] <<"]m, line_end: ["
                     << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
    return boost::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()));
  }

  // two circles
  if (model_name.compare("two_circles") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius")
        || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
    {
      ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                       << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    nh.getParam("footprint_model/front_offset", front_offset);
    nh.getParam("footprint_model/front_radius", front_radius);
    nh.getParam("footprint_model/rear_offset", rear_offset);
    nh.getParam("footprint_model/rear_radius", rear_radius);
    ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius
                    << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }

  // polygon
  if (model_name.compare("polygon") == 0)
  {

    // check parameters
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc) )
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                       << "/footprint_model/vertices' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get vertices
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      try
      {
        Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
        ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
        return boost::make_shared<PolygonRobotFootprint>(polygon);
      }
      catch(const std::exception& ex)
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
    }
    else
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                       << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }

  }

  // otherwise
  ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
  return boost::make_shared<PointRobotFootprint>();
}




Point2dContainer HATebLocalPlannerROS::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name)
{
   // Make sure we have an array of at least 3 elements.
   if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       footprint_xmlrpc.size() < 3)
   {
     ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
     throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                              "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
   }

   Point2dContainer footprint;
   Eigen::Vector2d pt;

   for (int i = 0; i < footprint_xmlrpc.size(); ++i)
   {
     // Make sure each element of the list is an array of size 2. (x and y coordinates)
     XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
     if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
         point.size() != 2)
     {
       ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                 "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                  full_param_name.c_str());
       throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x() = getNumberFromXMLRPC(point[ 0 ], full_param_name);
    pt.y() = getNumberFromXMLRPC(point[ 1 ], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

double HATebLocalPlannerROS::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str());
     throw std::runtime_error("Values in the footprint specification must be numbers");
   }
   return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

void HATebLocalPlannerROS::resetHumansPrediction() {
  std_srvs::Empty empty_service;
  ROS_INFO("Resetting human pose prediction");
  if (!reset_humans_prediction_client_ ||
      !reset_humans_prediction_client_.call(empty_service)) {
    ROS_WARN_THROTTLE(
        THROTTLE_RATE,
        "Failed to call %s service, is human prediction server running?",
        PREDICT_SERVICE_NAME);
    // re-initialize the service
    // reset_humans_prediction_client_ =
    //     nh.serviceClient<std_srvs::Empty>(RESET_PREDICTION_SERVICE_NAME,
    //     true);
  }
}

bool HATebLocalPlannerROS::optimizeStandalone(
    hateb_local_planner::Optimize::Request &req,
    hateb_local_planner::Optimize::Response &res) {
  ROS_INFO("optimize service called");
  auto start_time = ros::Time::now();

  // check if plugin initialized
  if (!initialized_) {
    res.success = false;
    res.message = "planner has not been initialized";
    return true;
  }

  auto trfm_start_time = ros::Time::now();
  // get robot pose from the costmap
  geometry_msgs::PoseStamped robot_pose_tf;
  costmap_ros_->getRobotPose(robot_pose_tf);

  // transform global plan to the frame of local costmap
  ROS_INFO("transforming robot global plans");
  PlanCombined transformed_plan_combined;
  int goal_idx;
  geometry_msgs::TransformStamped tf_robot_plan_to_global;

  if (!transformGlobalPlan(
          *tf_, req.robot_plan.poses, robot_pose_tf, *costmap_, global_frame_,
          cfg_.trajectory.max_global_plan_lookahead_dist, transformed_plan_combined,
          &goal_idx, &tf_robot_plan_to_global)) {
    res.success = false;
    res.message = "Could not transform the global plan to the local frame";
    return true;
  }
  auto &transformed_plan = transformed_plan_combined.plan_to_optimize;
  ROS_INFO("transformed plan contains %ld points (out of %ld)",
           transformed_plan.size(), req.robot_plan.poses.size());

  // check if the transformed robot plan is empty
  if (transformed_plan.empty()) {
    res.success = false;
    res.message = "Robot's transformed plan is empty";
    return true;
  }
  auto trfm_time = ros::Time::now() - trfm_start_time;

  // update obstacles container
  auto cc_start_time = ros::Time::now();
  obstacles_.clear();
  if (costmap_converter_)
    updateObstacleContainerWithCostmapConverter();
  else
    updateObstacleContainerWithCostmap();
  updateObstacleContainerWithCustomObstacles();
  auto cc_time = ros::Time::now() - cc_start_time;

  // update via-points container
  auto via_start_time = ros::Time::now();
  updateViaPointsContainer(transformed_plan,
                           cfg_.trajectory.global_plan_viapoint_sep);
  auto via_time = ros::Time::now() - via_start_time;

  // do not allow config changes from now until end of optimization
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

  // update humans
  auto human_start_time = ros::Time::now();

  HumanPlanVelMap transformed_human_plan_vel_map;
  std::vector<HumanPlanCombined> transformed_human_plans;
  tf2::Stamped<tf2::Transform> tf_human_plan_to_global;
  for (auto human_path : req.human_path_array.paths) {
    HumanPlanCombined human_plan_combined;
    geometry_msgs::TwistStamped transformed_vel;
    transformed_vel.header.frame_id = global_frame_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> human_path_cov;
    for (auto human_pose : human_path.path.poses) {
      geometry_msgs::PoseWithCovarianceStamped human_pos_cov;
      human_pos_cov.header = human_pose.header;
      human_pos_cov.pose.pose = human_pose.pose;
      human_path_cov.push_back(human_pos_cov);
    }
    ROS_INFO("transforming human %ld plan", human_path.id);
    if (!transformHumanPlan(*tf_, robot_pose_tf, *costmap_, global_frame_,
                            human_path_cov, human_plan_combined,
                            transformed_vel, &tf_human_plan_to_global)) {
      res.success = false;
      res.message = "could not transform human" +
                    std::to_string(human_path.id) + " plan to the local frame";
      return true;
    }
    auto transformed_plan_size = human_plan_combined.plan_before.size() +
                                 human_plan_combined.plan_to_optimize.size() +
                                 human_plan_combined.plan_after.size();
    ROS_INFO("transformed human %ld plan contains %ld (before %ld, "
             "to-optimize %ld, after %ld) points (out of %ld (%ld))",
             human_path.id, transformed_plan_size,
             human_plan_combined.plan_before.size(),
             human_plan_combined.plan_to_optimize.size(),
             human_plan_combined.plan_after.size(), human_path_cov.size(),
             human_path.path.poses.size());
    // TODO: check for empty human transformed plan

    human_plan_combined.id = human_path.id;
    transformed_human_plans.push_back(human_plan_combined);

    PlanStartVelGoalVel plan_start_vel_goal_vel;
    plan_start_vel_goal_vel.plan = human_plan_combined.plan_to_optimize;
    plan_start_vel_goal_vel.start_vel = transformed_vel.twist;
    // plan_start_vel_goal_vel.nominal_vel = std::max(0.3,human_nominal_vels[predicted_humans_poses.id-1]);
    plan_start_vel_goal_vel.isMode = isMode;
    if (human_plan_combined.plan_after.size() > 0) {
      plan_start_vel_goal_vel.goal_vel = transformed_vel.twist;
    }
    transformed_human_plan_vel_map[human_plan_combined.id] =
        plan_start_vel_goal_vel;
  }

  updateHumanViaPointsContainers(transformed_human_plan_vel_map,
                                 cfg_.trajectory.global_plan_viapoint_sep);
  auto human_time = ros::Time::now() - human_start_time;

  // now perform the actual planning
  auto plan_start_time = ros::Time::now();
  geometry_msgs::Twist robot_vel_twist;
  hateb_local_planner::OptimizationCostArray op_costs;

  bool success = planner_->plan(transformed_plan, &robot_vel_,
                                cfg_.goal_tolerance.free_goal_vel,
                                &transformed_human_plan_vel_map,&op_costs);
  if (!success) {
    planner_->clearPlanner();
    res.success = false;
    res.message =
        "planner was not able to obtain a local plan for the current setting";
    return true;
  }
  // op_costs_pub_.publish(op_costs);

  auto plan_time = ros::Time::now() - plan_start_time;
  auto viz_start_time = ros::Time::now();

  PlanTrajCombined plan_traj_combined;
  plan_traj_combined.plan_before = transformed_plan_combined.plan_before;
  planner_->getFullTrajectory(plan_traj_combined.optimized_trajectory);
  plan_traj_combined.plan_after = transformed_plan_combined.plan_after;
  visualization_->publishTrajectory(plan_traj_combined);

  std::vector<HumanPlanTrajCombined> human_plans_traj_array;
  for (auto &human_plan_combined : transformed_human_plans) {
    HumanPlanTrajCombined human_plan_traj_combined;
    human_plan_traj_combined.id = human_plan_combined.id;
    human_plan_traj_combined.plan_before = human_plan_combined.plan_before;
    planner_->getFullHumanTrajectory(
        human_plan_traj_combined.id,
        human_plan_traj_combined.optimized_trajectory);
    human_plan_traj_combined.plan_after = human_plan_combined.plan_after;
    human_plans_traj_array.push_back(human_plan_traj_combined);
  }
  visualization_->publishHumanTrajectories(human_plans_traj_array);
  // now visualize everything
  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  visualization_->publishHumanGlobalPlans(transformed_human_plans);

  auto viz_time = ros::Time::now() - viz_start_time;

  res.success = true;
  res.message = "planning successful";
  geometry_msgs::Twist cmd_vel;



  // check feasibility of robot plan
  auto fsb_start_time = ros::Time::now();
  bool feasible = planner_->isTrajectoryFeasible(
      costmap_model_.get(), footprint_spec_, robot_inscribed_radius_,
      robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses);
  if (!feasible) {
    res.message += "\nhowever, trajectory is not feasible";
  }
  auto fsb_time = ros::Time::now() - fsb_start_time;

  // get the velocity command for this sampling interval
  auto vel_start_time = ros::Time::now();
  double dt_resize = 0.4;
  if (!planner_->getVelocityCommand(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cfg_.trajectory.control_look_ahead_poses, dt_resize)) {
    res.message += feasible ? "\nhowever," : "\nand";
    res.message += " velocity command is invalid";
  }

  // clear the planner only after getting the velocity command
  planner_->clearPlanner();

  // saturate velocity
  saturateVelocity(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cfg_.robot.max_vel_x, cfg_.robot.max_vel_y,
                   cfg_.robot.max_vel_theta, cfg_.robot.max_vel_x_backwards);

  auto vel_time = ros::Time::now() - vel_start_time;

  auto total_time = ros::Time::now() - start_time;

  res.message += "\ncompute velocity times:";
  res.message +=
      "\n\ttotal time                  " + std::to_string(total_time.toSec()) +
      "\n\ttransform time              " + std::to_string(trfm_time.toSec()) +
      "\n\tcostmap convert time        " + std::to_string(cc_time.toSec()) +
      "\n\tvia points time             " + std::to_string(via_time.toSec()) +
      "\n\thuman time                  " + std::to_string(human_time.toSec()) +
      "\n\tplanning time               " + std::to_string(plan_time.toSec()) +
      "\n\tplan feasibility check time " + std::to_string(fsb_time.toSec()) +
      "\n\tvelocity extract time       " + std::to_string(vel_time.toSec()) +
      "\n\tvisualization publish time  " + std::to_string(viz_time.toSec()) +
      "\n=================================";
  return true;
}

bool HATebLocalPlannerROS::setApproachID(
    hateb_local_planner::Approach::Request &req,
    hateb_local_planner::Approach::Response &res) {
  if (cfg_.planning_mode == 2) {
    cfg_.approach.approach_id = req.human_id;
    res.message +=
        "Approach ID set to " + std::to_string(cfg_.approach.approach_id);
    res.success = true;
  } else {
    cfg_.approach.approach_id = -1;
    res.message = "No approach ID set, planner is not running in approach mode";
    res.success = false;
  }
  return true;
}

} // end namespace hateb_local_planner
