/*********************************************************************
 *
 * Software License Agreement (BSD License)
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
 *  Authors: Harmish Khambhaita, Phani Teja Singamaneni
 *********************************************************************/

// defining constants
#define NODE_NAME "human_path_prediction"
#define HUMANS_SUB_TOPIC "tracked_humans"
#define EXTERNAL_PATHS_SUB_TOPIC "external_human_paths"
#define RESET_EXT_PATHS_SERVICE_NAME "reset_external_paths"
#define PREDICT_SERVICE_NAME "predict_human_poses"
#define PUBLISH_MARKERS_SRV_NAME "publish_prediction_markers"
#define PREDICTED_HUMANS_MARKERS_PUB_TOPIC "predicted_human_poses"
#define GET_PLAN_SRV_NAME "/move_base/GlobalPlanner/make_plan"
#define DEFAULT_HUMAN_PART human_msgs::TrackedSegmentType::TORSO
#define MAX_HUMAN_MARKERS 1000
#define MIN_MARKER_LIFETIME 1.0
#define MINIMUM_COVARIANCE_MARKERS 0.1
#define ROBOT_FRAME_ID "base_footprint"
#define MAP_FRAME_ID "map"
#define HUMAN_DIST_BEHIND_ROBOT 1.0 // in meters
#define HUMAN_ANGLE_BEHIND_ROBOT 3.14
#define ANG_VEL_EPS 0.001
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <signal.h>

#include <human_path_prediction/human_path_prediction.h>
#include <std_srvs/Empty.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>
#include <std_srvs/Trigger.h>

namespace human_path_prediction {
// empty constructor and destructor
HumanPathPrediction::HumanPathPrediction() {}
HumanPathPrediction::~HumanPathPrediction() {}

void HumanPathPrediction::initialize() {
  // get private node handle
  ros::NodeHandle private_nh("~/");

  // get parameters
  private_nh.param("tracked_humans_sub_topic", tracked_humans_sub_topic_,
                   std::string(HUMANS_SUB_TOPIC));
  private_nh.param("external_paths_sub_topic", external_paths_sub_topic_,
                   std::string(EXTERNAL_PATHS_SUB_TOPIC));
  private_nh.param("reset_ext_paths_service_name",
                   reset_ext_paths_service_name_,
                   std::string(RESET_EXT_PATHS_SERVICE_NAME));
  private_nh.param("predict_service_name", predict_service_name_,
                   std::string(PREDICT_SERVICE_NAME));
  private_nh.param("predicted_humans_markers_pub_topic",
                   predicted_humans_markers_pub_topic_,
                   std::string(PREDICTED_HUMANS_MARKERS_PUB_TOPIC));
  private_nh.param("get_plan_srv_name", get_plan_srv_name_,
                   std::string(GET_PLAN_SRV_NAME));
  private_nh.param("publish_markers_srv_name", publish_markers_srv_name_,
                   std::string(PUBLISH_MARKERS_SRV_NAME));
  private_nh.param("default_human_part", default_human_part_,
                   (int)(DEFAULT_HUMAN_PART));
  private_nh.param("robot_frame_id", robot_frame_id_,
                   std::string(ROBOT_FRAME_ID));
  private_nh.param("map_frame_id", map_frame_id_, std::string(MAP_FRAME_ID));
  private_nh.param("human_dist_behind_robot", human_dist_behind_robot_,
                   HUMAN_DIST_BEHIND_ROBOT);
  private_nh.param("human_angle_behind_robot", human_angle_behind_robot_,
                   HUMAN_ANGLE_BEHIND_ROBOT);

  // initialize subscribers and publishers
  tracked_humans_sub_ =
      private_nh.subscribe(tracked_humans_sub_topic_, 1,
                           &HumanPathPrediction::trackedHumansCB, this);
  external_paths_sub_ =
      private_nh.subscribe(external_paths_sub_topic_, 1,
                           &HumanPathPrediction::externalPathsCB, this);
  external_trajs_sub_ = private_nh.subscribe("external_human_trajs", 1, &HumanPathPrediction::externalTrajsCB, this);

  predicted_goal_sub_ = private_nh.subscribe("/human_goal_predict/predicted_goal", 1, &HumanPathPrediction::predictedGoalCB, this);

  predicted_humans_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(
      predicted_humans_markers_pub_topic_, 1);

  // set-up dynamic reconfigure
  dsrv_ =
      new dynamic_reconfigure::Server<HumanPathPredictionConfig>(private_nh);
  dynamic_reconfigure::Server<HumanPathPredictionConfig>::CallbackType cb =
      boost::bind(&HumanPathPrediction::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  // initialize services
  predict_humans_server_ = private_nh.advertiseService(predict_service_name_, &HumanPathPrediction::predictHumans, this);
  reset_ext_paths_server_ = private_nh.advertiseService(reset_ext_paths_service_name_, &HumanPathPrediction::resetExtPaths, this);
  publish_markers_srv_ = private_nh.advertiseService(publish_markers_srv_name_, &HumanPathPrediction::setPublishMarkers, this);
  get_plan_client_ = private_nh.serviceClient<nav_msgs::GetPlan>(get_plan_srv_name_, true);
  set_goal_srv_ = private_nh.advertiseService("set_human_goal", &HumanPathPrediction::setExternalGoal, this);
  set_goal_call_srv_ = private_nh.advertiseService("check_human_goal", &HumanPathPrediction::checkExternalGoal, this);
  goal_change_srv_  = private_nh.serviceClient<std_srvs::Trigger>("/goal_changed");

  showing_markers_ = false;
  got_new_human_paths_ = false;
  done_cfg =  false;
  got_external_goal = false;

  ROS_DEBUG_NAMED(NODE_NAME, "node %s initialized", NODE_NAME);
}

void HumanPathPrediction::setParams(std::vector<double> velscale_scales,
                                    double velscale_angle, double velscale_mul,
                                    double velobs_mul, double velobs_min_rad,
                                    double velobs_max_rad,
                                    double velobs_max_rad_time,
                                    bool velobs_use_ang) {
  velscale_scales_ = velscale_scales;
  velscale_angle_ = velscale_angle;
  velscale_mul_ = velscale_mul;
  velobs_mul_ = velobs_mul;
  velobs_min_rad_ = velobs_min_rad;
  velobs_max_rad_ = velobs_max_rad;
  velobs_max_rad_time_ = velobs_max_rad_time;
  velobs_use_ang_ = velobs_use_ang;

  ROS_DEBUG_NAMED(NODE_NAME, "parameters set: velocity-scale: scales=[%f, %f, "
                             "%f], angle=%f, velscale-mul=%f, velobs-mul=%f "
                             "velocity-obstacle: min-radius:%f, max-radius:%f, "
                             "max-radius-time=%f use-ang=%d",
                  velscale_scales_[0], velscale_scales_[1], velscale_scales_[2],
                  velscale_angle_, velscale_mul_, velobs_mul_, velobs_min_rad_,
                  velobs_max_rad_, velobs_max_rad_time_, velobs_use_ang_);
}

void HumanPathPrediction::reconfigureCB(HumanPathPredictionConfig &config,
                                        uint32_t level) {
  setParams(
      {config.velscale_lower, config.velscale_nominal, config.velscale_higher},
      config.velscale_angle, config.velscale_mul, config.velobs_mul,
      config.velobs_min_rad, config.velobs_max_rad, config.velobs_max_rad_time,
      config.velobs_use_ang);
}

void HumanPathPrediction::trackedHumansCB(
    const human_msgs::TrackedHumans &tracked_humans) {
  ROS_INFO_ONCE_NAMED(NODE_NAME, "human_path_prediction: received humans");
  tracked_humans_ = tracked_humans;
}

double HumanPathPrediction::checkdist(geometry_msgs::Pose human, geometry_msgs::Pose robot){
  return std::hypot(human.position.x-robot.position.x,human.position.y-robot.position.y);
}

void HumanPathPrediction::externalPathsCB(
    const human_msgs::HumanPathArray::ConstPtr &external_paths) {
  ROS_INFO_ONCE_NAMED(NODE_NAME, "human_path_prediction: received human paths");
  external_paths_ = external_paths;
  got_new_human_paths_ = true;
}

void HumanPathPrediction::predictedGoalCB(const human_path_prediction::PredictedGoal::ConstPtr &predicted_goal){
  ROS_INFO_ONCE_NAMED(NODE_NAME, "human_path_prediction: received predicted goal");
  predicted_goal_ = predicted_goal;
  // got_new_goal = true;
}


void HumanPathPrediction::externalTrajsCB(
    const human_msgs::HumanTrajectoryArrayConstPtr &traj_array) {
  boost::mutex::scoped_lock(external_trajs_mutex_);
  human_msgs::HumanPathArray external_paths;
  external_paths.header = traj_array->header;

  for(auto &trajs : traj_array->trajectories){
    human_msgs::HumanPath path;
    nav_msgs::Path nav_path;

    path.header = trajs.header;
    path.id = trajs.id;

    for(auto &point : trajs.trajectory.points){
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = point.transform.translation.x;
      pose.pose.position.y = point.transform.translation.y;
      pose.pose.position.z = point.transform.translation.z;
      pose.pose.orientation = point.transform.rotation;
      nav_path.poses.push_back(pose);
    }
    path.path = nav_path;
    external_paths.paths.push_back(path);
  }

  external_paths2_ = external_paths;
}

bool HumanPathPrediction::predictHumans(
    human_path_prediction::HumanPosePredict::Request &req,
    human_path_prediction::HumanPosePredict::Response &res) {
  boost::function<bool(human_path_prediction::HumanPosePredict::Request & req,
                       human_path_prediction::HumanPosePredict::Response & res)>
      prediction_function;

  switch (req.type) {
  case human_path_prediction::HumanPosePredictRequest::VELOCITY_SCALE:
    prediction_function =
        boost::bind(&HumanPathPrediction::predictHumansVelScale, this, _1, _2);
    break;
  case human_path_prediction::HumanPosePredictRequest::VELOCITY_OBSTACLE:
    prediction_function =
        boost::bind(&HumanPathPrediction::predictHumansVelObs, this, _1, _2);
    break;
  case human_path_prediction::HumanPosePredictRequest::EXTERNAL:
    prediction_function =
        boost::bind(&HumanPathPrediction::predictHumansExternal, this, _1, _2);
    break;
  case human_path_prediction::HumanPosePredictRequest::BEHIND_ROBOT:
    prediction_function =
        boost::bind(&HumanPathPrediction::predictHumansBehind, this, _1, _2);
    break;
  case human_path_prediction::HumanPosePredictRequest::PREDICTED_GOAL:
    prediction_function =
        boost::bind(&HumanPathPrediction::predictHumansGoal, this, _1, _2);
    break;
  default:
    ROS_ERROR_NAMED(NODE_NAME, "%s: unkonwn prediction type %d", NODE_NAME,
                    req.type);
  }

  if (!prediction_function.empty() && prediction_function(req, res)) {
    if (publish_markers_) {
      // create new markers
      predicted_humans_markers_.markers.clear();

      for (auto predicted_human : res.predicted_humans_poses) {
        if (predicted_human.poses.size() > 0) {
          auto first_pose_time = predicted_human.poses[0].header.stamp;
          int marker_id = 0;

          for (auto predicted_human_pose : predicted_human.poses) {
            visualization_msgs::Marker predicted_human_marker;
            predicted_human_marker.header.frame_id =
                predicted_human_pose.header.frame_id;
            predicted_human_marker.header.stamp = first_pose_time;
            predicted_human_marker.id =
                (predicted_human.id * MAX_HUMAN_MARKERS) + marker_id++;
            predicted_human_marker.type = visualization_msgs::Marker::CYLINDER;
            predicted_human_marker.action = visualization_msgs::Marker::MODIFY;
            // assuming diagonal covariance matrix (with row-major order)
            predicted_human_marker.scale.x =
                std::max(predicted_human_pose.pose.covariance[0],
                         MINIMUM_COVARIANCE_MARKERS);
            predicted_human_marker.scale.y =
                std::max(predicted_human_pose.pose.covariance[7],
                         MINIMUM_COVARIANCE_MARKERS);
            predicted_human_marker.scale.z = 0.01;
            predicted_human_marker.color.a = 1.0;
            predicted_human_marker.color.r = 0.0;
            predicted_human_marker.color.g = 0.0;
            predicted_human_marker.color.b = 1.0;
            predicted_human_marker.lifetime =
                ros::Duration(MIN_MARKER_LIFETIME) +
                (predicted_human_pose.header.stamp - first_pose_time);
            predicted_human_marker.pose.position.x =
                predicted_human_pose.pose.pose.position.x;
            predicted_human_marker.pose.position.y =
                predicted_human_pose.pose.pose.position.y;
            // time on z axis
            predicted_human_marker.pose.position.z =
                (predicted_human_pose.header.stamp - first_pose_time).toSec();
            predicted_humans_markers_.markers.push_back(predicted_human_marker);
          }

          auto it = last_markers_size_map.find(predicted_human.id);
          if (it != last_markers_size_map.end()) {
            while (it->second >= marker_id) {
              visualization_msgs::Marker delete_human_marker;
              delete_human_marker.id =
                  (predicted_human.id * MAX_HUMAN_MARKERS) + marker_id++;
              delete_human_marker.action = visualization_msgs::Marker::DELETE;
              predicted_humans_markers_.markers.push_back(delete_human_marker);
            }
          }
          last_markers_size_map[predicted_human.id] = --marker_id;
        } else {
          ROS_WARN_NAMED(NODE_NAME, "no predicted poses fro human %ld",
                         predicted_human.id);
        }
      }

      predicted_humans_pub_.publish(predicted_humans_markers_);
      showing_markers_ = true;

      ROS_DEBUG_NAMED(NODE_NAME, "published predicted humans");
    } else {
      if (showing_markers_) {
        predicted_humans_markers_.markers.clear();
        visualization_msgs::Marker delete_human_markers;
        delete_human_markers.action =
            3; // visualization_msgs::Marker::DELETEALL;
        predicted_humans_markers_.markers.push_back(delete_human_markers);
        predicted_humans_pub_.publish(predicted_humans_markers_);
        showing_markers_ = false;
      }
    }

    return true;
  } else {
    return false;
  }
}

bool HumanPathPrediction::predictHumansVelScale(
    human_path_prediction::HumanPosePredict::Request &req,
    human_path_prediction::HumanPosePredict::Response &res) {
  // validate prediction time
  if (req.predict_times.size() == 0) {
    ROS_ERROR_NAMED(NODE_NAME, "prediction times cannot be empty");
    return false;
  }
  if (req.predict_times[0] < 0) {
    ROS_ERROR_NAMED(NODE_NAME, "prediction time cannot be negative (give %f)",
                    req.predict_times[0]);
    return false;
  }

  // get local refrence of humans
  auto humans = tracked_humans_.humans;
  auto track_frame = tracked_humans_.header.frame_id;
  auto track_time = tracked_humans_.header.stamp;

  if (track_time.toSec() < req.predict_times[0]) {
    ROS_DEBUG_NAMED(
        NODE_NAME,
        "human data is older than prediction time, predicting nothing");
    return true;
  }

  for (auto human : humans) {
    // TODO: filter by res.ids

    for (auto segment : human.segments) {
      if (segment.type == default_human_part_) {
        // get linear velocity of the human
        tf::Vector3 linear_vel(segment.twist.twist.linear.x,
                               segment.twist.twist.linear.y,
                               segment.twist.twist.linear.z);

        // calculate variations in velocity of human
        std::vector<tf::Vector3> vel_variations;
        for (auto vel_scale : velscale_scales_) {
          vel_variations.push_back(
              linear_vel.rotate(tf::Vector3(0, 0, 1), velscale_angle_) *
              vel_scale * velscale_mul_);
          vel_variations.push_back(
              linear_vel.rotate(tf::Vector3(0, 0, 1), -velscale_angle_) *
              vel_scale * velscale_mul_);
        }

        // calculate future human poses based on velocity variations
        human_path_prediction::PredictedPoses predicted_poses;
        predicted_poses.id = human.track_id;
        for (auto vel : vel_variations) {
          geometry_msgs::PoseWithCovarianceStamped predicted_pose;
          predicted_pose.header.frame_id = track_frame;
          predicted_pose.header.stamp =
              track_time + ros::Duration(req.predict_times[0]);
          predicted_pose.pose.pose.position.x =
              segment.pose.pose.position.x + vel.x() * req.predict_times[0];
          predicted_pose.pose.pose.position.y =
              segment.pose.pose.position.y + vel.y() * req.predict_times[0];
          predicted_pose.pose.pose.orientation = segment.pose.pose.orientation;
          // no covariance for this method
          predicted_poses.poses.push_back(predicted_pose);

          ROS_DEBUG_NAMED(
              NODE_NAME, "predected human (%lu) segment (%d)"
                         " pose: x=%f, y=%f, theta=%f with vel x=%f,y=%f",
              human.track_id, segment.type, predicted_pose.pose.pose.position.x,
              predicted_pose.pose.pose.position.y,
              tf::getYaw(predicted_pose.pose.pose.orientation), vel.x(),
              vel.y());
        }

        geometry_msgs::TwistStamped current_twist;
        current_twist.header.frame_id = track_frame;
        current_twist.header.stamp = track_time;
        current_twist.twist = segment.twist.twist;
        predicted_poses.start_velocity = current_twist;

        res.predicted_humans_poses.push_back(predicted_poses);
      }
    }
  }

  return true;
}

bool HumanPathPrediction::predictHumansVelObs(
    human_path_prediction::HumanPosePredict::Request &req,
    human_path_prediction::HumanPosePredict::Response &res) {
  // validate prediction time
  if (req.predict_times.size() == 0) {
    ROS_ERROR_NAMED(NODE_NAME, "prediction times cannot be empty");
    return false;
  }
  if (*std::min_element(req.predict_times.begin(), req.predict_times.end()) <
      0.0) {
    ROS_ERROR_NAMED(NODE_NAME, "prediction time cannot be negative");
    return false;
  }

  // get local refrence of humans
  auto humans = tracked_humans_.humans;
  auto track_frame = tracked_humans_.header.frame_id;
  auto track_time = tracked_humans_.header.stamp;

  if ((ros::Time::now() - track_time).toSec() >
      *std::max_element(req.predict_times.begin(), req.predict_times.end())) {
    ROS_DEBUG_NAMED(NODE_NAME, "human data is older than maximum given "
                               "prediction time, predicting nothing");
    return true;
  }

  for (auto human : humans) {
    // TODO: filter by res.ids
    if(std::find(req.ids.begin(),req.ids.end(),human.track_id)==req.ids.end()){
      continue;
    }
    for (auto segment : human.segments) {
      if (segment.type == default_human_part_) {
        // calculate future human poses based on current velocity
        human_path_prediction::PredictedPoses predicted_poses;
        predicted_poses.id = human.track_id;

        // get linear velocity of the human
        tf::Vector3 linear_vel(segment.twist.twist.linear.x,
                               segment.twist.twist.linear.y,
                               segment.twist.twist.linear.z);

        for (auto predict_time : req.predict_times) {
          // validate prediction time
          if (predict_time < 0) {
            ROS_ERROR_NAMED(NODE_NAME,
                            "%s: prediction time cannot be negative (give %f)",
                            NODE_NAME, predict_time);
            return false;
          }

          geometry_msgs::PoseWithCovarianceStamped predicted_pose;
          predicted_pose.header.frame_id = track_frame;
          predicted_pose.header.stamp =
              track_time + ros::Duration(predict_time);

          if (velobs_use_ang_ &&
              std::abs(segment.twist.twist.angular.z) > ANG_VEL_EPS) {
            // velocity multiplier is only applied to linear velocities
            double r =
                (std::hypot(linear_vel[0], linear_vel[1]) * velobs_mul_) /
                segment.twist.twist.angular.z;
            double theta = segment.twist.twist.angular.z * predict_time;
            double crd = r * 2 * std::sin(theta / 2);
            double alpha =
                std::atan2(linear_vel[1], linear_vel[0]) + (theta / 2);
            predicted_pose.pose.pose.position.x =
                segment.pose.pose.position.x + crd * std::cos(alpha);
            predicted_pose.pose.pose.position.y =
                segment.pose.pose.position.y + crd * std::sin(alpha);
            predicted_pose.pose.pose.orientation =
                tf::createQuaternionMsgFromYaw(
                    tf::getYaw(segment.pose.pose.orientation) + theta);
          } else {
            predicted_pose.pose.pose.position.x =
                segment.pose.pose.position.x +
                linear_vel[0] * predict_time * velobs_mul_;
            predicted_pose.pose.pose.position.y =
                segment.pose.pose.position.y +
                linear_vel[1] * predict_time * velobs_mul_;
            predicted_pose.pose.pose.orientation =
                segment.pose.pose.orientation;
          }

          // not using velocity multiplier for covariance matrix
          double xy_vel =
              hypot(linear_vel[0] * predict_time, linear_vel[1] * predict_time);
          // storing only x, y covariance in diagonal matrix
          predicted_pose.pose.covariance[0] =
              velobs_min_rad_ +
              (velobs_max_rad_ - velobs_min_rad_) *
                  (predict_time / velobs_max_rad_time_) * xy_vel;
          predicted_pose.pose.covariance[7] = predicted_pose.pose.covariance[0];
          predicted_poses.poses.push_back(predicted_pose);

          ROS_DEBUG_NAMED(
              NODE_NAME, "%s: predected human (%lu) segment (%d)"
                         " pose: x=%f, y=%f, theta=%f, predict-time=%f",
              NODE_NAME, human.track_id, segment.type,
              predicted_pose.pose.pose.position.x,
              predicted_pose.pose.pose.position.y,
              tf::getYaw(predicted_pose.pose.pose.orientation), predict_time);
        }

        geometry_msgs::TwistStamped current_twist;
        current_twist.header.frame_id = track_frame;
        current_twist.header.stamp = track_time;
        current_twist.twist = segment.twist.twist;
        predicted_poses.start_velocity = current_twist;

        res.predicted_humans_poses.push_back(predicted_poses);
      }
    }
  }

  return true;
}

bool HumanPathPrediction::predictHumansExternal(
    human_path_prediction::HumanPosePredict::Request &req,
    human_path_prediction::HumanPosePredict::Response &res) {
  if (external_paths_) {
    auto external_paths = external_paths_;
    auto tracked_humans = tracked_humans_;

    std::vector<HumanPathVel> human_path_vel_array;
    for (auto &path : external_paths->paths) {
      HumanPathVel human_path_vel{.id = path.id, .path = path.path};

      // set starting velocity of the human if we find them
      // we do not add current pose at first pose in this case
      for (auto &human : tracked_humans.humans) {
        if (human.track_id == path.id) {
          for (auto &segment : human.segments) {
            if (segment.type == default_human_part_) {
              human_path_vel.start_vel = segment.twist;
              break;
            }
          }
          break;
        }
      }
      human_path_vel_array.push_back(human_path_vel);
    }
    return predictHumansFromPaths(req, res, human_path_vel_array);
  }
  else if(got_external_goal){
    auto now = ros::Time::now();
    auto tracked_humans = tracked_humans_;
    std::map<uint64_t, geometry_msgs::PoseStamped> ext_goal;

    // get robot pose
    tf::StampedTransform robot_to_map_tf, human_to_map_tf;
    bool transforms_found = false;
    try {
      tf_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0),
                          robot_to_map_tf);
      tf_.lookupTransform(map_frame_id_, tracked_humans.header.frame_id,
                          ros::Time(0), human_to_map_tf);
      transforms_found = true;
    } catch (tf::LookupException &ex) {
      ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n",
                      ex.what());
    } catch (tf::ConnectivityException &ex) {
      ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
    } catch (tf::ExtrapolationException &ex) {
      ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
    }

    // first check if path calculation is needed, and for whom
    std::vector<HumanStartPoseVel> human_start_pose_vels;
    std::vector<bool> start_poses_far;
    int idx_order = 0;
    for (auto &human : tracked_humans.humans) {
      behind_path_vels_pos.push_back(-1);
      if(std::find(req.ids.begin(),req.ids.end(),human.track_id)==req.ids.end()){
        continue;
      }
      bool path_exist = false;
      for(auto &ex_gl : external_goals_){
      	if(ex_gl.id == human.track_id){
      	  ext_goal[ex_gl.id] = ex_gl.pose;
      	  break;
	       }
      }
      for (auto path_vel : behind_path_vels_) {
        if (path_vel.id == human.track_id) {
          path_exist = true;
          break;
        }
      }

      // get human pose
      for (auto &segment : human.segments) {
        if (segment.type == default_human_part_) {
          geometry_msgs::PoseStamped human_start;
          human_start.header.frame_id = tracked_humans.header.frame_id;
          human_start.header.stamp = now;
          human_start.pose = segment.pose.pose;

          tf::Pose start_pose_tf;
          geometry_msgs::Pose start_pose_;
          tf::poseMsgToTF(human_start.pose, start_pose_tf);
          start_pose_tf = human_to_map_tf * start_pose_tf;
          tf::poseTFToMsg(start_pose_tf, start_pose_);

          if(!path_exist){
            HumanStartPoseVel human_start_pose_vel = {
                .id = human.track_id, .pose = human_start, .vel = segment.twist};
            human_start_pose_vels.push_back(human_start_pose_vel);
            behind_path_vels_pos[human.track_id-1] = idx_order;
          }
          else{
            if(behind_path_vels_.size()>=human.track_id && std::find(req.ids.begin(),req.ids.end(),human.track_id)!=req.ids.end()){
              double dist_far = std::hypot(human_start.pose.position.x-behind_path_vels_[behind_path_vels_pos[human.track_id-1]].path.poses[0].pose.position.x,
                                           human_start.pose.position.y-behind_path_vels_[behind_path_vels_pos[human.track_id-1]].path.poses[0].pose.position.y);
            if(dist_far > 0.5){ // To ensure that the path is recalculated only if the human is deviating from the path
              start_poses_far.push_back(true);
              HumanStartPoseVel human_start_pose_vel = {
                  .id = human.track_id, .pose = human_start, .vel = segment.twist};
              human_start_pose_vels.push_back(human_start_pose_vel);
              behind_path_vels_pos[human.track_id-1] = idx_order;
              behind_path_vels_.clear();
            }
           }
          }
          break;
        }
      }
      idx_order++;
    }

    if (!human_start_pose_vels.empty()) {

      if (transforms_found) {
        for (auto &human_start_pose_vel : human_start_pose_vels) {
          nav_msgs::GetPlan get_plan_srv;
          if(ext_goal.find(human_start_pose_vel.id) == ext_goal.end())
            continue;
          // get human pose in map frame
          tf::Pose start_pose_tf;
          tf::poseMsgToTF(human_start_pose_vel.pose.pose, start_pose_tf);
          start_pose_tf = human_to_map_tf * start_pose_tf;
          get_plan_srv.request.start.header.frame_id = map_frame_id_;
          get_plan_srv.request.start.header.stamp = now;
          tf::poseTFToMsg(start_pose_tf, get_plan_srv.request.start.pose);

          get_plan_srv.request.goal.header.frame_id = map_frame_id_;
          get_plan_srv.request.goal.header.stamp = now;
          get_plan_srv.request.goal.pose.position.x = ext_goal[human_start_pose_vel.id].pose.position.x;
          get_plan_srv.request.goal.pose.position.y = ext_goal[human_start_pose_vel.id].pose.position.y;
          get_plan_srv.request.goal.pose.position.z = ext_goal[human_start_pose_vel.id].pose.position.z;
          get_plan_srv.request.goal.pose.orientation = ext_goal[human_start_pose_vel.id].pose.orientation;

          ROS_DEBUG_NAMED(NODE_NAME, "human start: x=%.2f, y=%.2f, theta=%.2f, "
                                     "goal: x=%.2f, y=%.2f, theta=%.2f",
                          get_plan_srv.request.start.pose.position.x,
                          get_plan_srv.request.start.pose.position.y,
                          tf::getYaw(get_plan_srv.request.start.pose.orientation),
                          get_plan_srv.request.goal.pose.position.x,
                          get_plan_srv.request.goal.pose.position.y,
                          tf::getYaw(get_plan_srv.request.goal.pose.orientation));

          // make plan for human
          if (get_plan_client_) {
            if (get_plan_client_.call(get_plan_srv)) {
              if (get_plan_srv.response.plan.poses.size() > 0) {
                HumanPathVel human_path_vel;
                human_path_vel.id = human_start_pose_vel.id;
                human_path_vel.path = get_plan_srv.response.plan;
                human_path_vel.start_vel = human_start_pose_vel.vel;
                behind_path_vels_.push_back(human_path_vel);
                got_new_human_paths_ = true;
              } else {
                ROS_WARN_NAMED(NODE_NAME, "Got empty path for human, start or "
                                          "goal position is probably invalid");
              }
            } else {
              ROS_WARN_NAMED(NODE_NAME, "Failed to call %s service",
                             get_plan_srv_name_.c_str());
            }
          } else {
            ROS_WARN_NAMED(NODE_NAME,
                           "%s service does not exist, re-trying to subscribe",
                           get_plan_srv_name_.c_str());
            ros::NodeHandle private_nh("~/");
            get_plan_client_ = private_nh.serviceClient<nav_msgs::GetPlan>(
                get_plan_srv_name_, true);
          }
        }
      }
    }
    return predictHumansFromPaths(req, res, behind_path_vels_);
  }

  else {
    std::vector<HumanPathVel> empty_path_vels;
    return predictHumansFromPaths(req, res, empty_path_vels);
  }
}


bool HumanPathPrediction::predictHumansBehind(
    human_path_prediction::HumanPosePredict::Request &req,
    human_path_prediction::HumanPosePredict::Response &res) {
  auto now = ros::Time::now();
  auto tracked_humans = tracked_humans_;

  // get robot pose
  tf::StampedTransform robot_to_map_tf, human_to_map_tf;
  bool transforms_found = false;
  try {
    tf_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0),
                        robot_to_map_tf);
    tf_.lookupTransform(map_frame_id_, tracked_humans.header.frame_id,
                        ros::Time(0), human_to_map_tf);
    transforms_found = true;
  } catch (tf::LookupException &ex) {
    ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n",
                    ex.what());
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
  } catch (tf::ExtrapolationException &ex) {
    ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
  }

  // first check if path calculation is needed, and for whom
  std::vector<HumanStartPoseVel> human_start_pose_vels;
  std::vector<bool> start_poses_far;
  int idx_order = 0;
  for (auto &human : tracked_humans.humans) {
    behind_path_vels_pos.push_back(-1);
    if(std::find(req.ids.begin(),req.ids.end(),human.track_id)==req.ids.end()){
      continue;
    }
    bool path_exist = false;
    for (auto path_vel : behind_path_vels_) {
      if (path_vel.id == human.track_id) {
        path_exist = true;
        break;
      }
    }

      // get human pose
      for (auto &segment : human.segments) {
        if (segment.type == default_human_part_) {
          geometry_msgs::PoseStamped human_start;
          human_start.header.frame_id = tracked_humans.header.frame_id;
          human_start.header.stamp = now;
          human_start.pose = segment.pose.pose;

          tf::Pose start_pose_tf;
          geometry_msgs::Pose start_pose_;
          tf::poseMsgToTF(human_start.pose, start_pose_tf);
          start_pose_tf = human_to_map_tf * start_pose_tf;
          tf::poseTFToMsg(start_pose_tf, start_pose_);

          if(!path_exist){
          HumanStartPoseVel human_start_pose_vel = {
              .id = human.track_id, .pose = human_start, .vel = segment.twist};
          human_start_pose_vels.push_back(human_start_pose_vel);
          behind_path_vels_pos[human.track_id-1] = idx_order;
          }
          else{
            if(behind_path_vels_.size()>=human.track_id && std::find(req.ids.begin(),req.ids.end(),human.track_id)!=req.ids.end()){
            double dist_far = std::hypot(human_start.pose.position.x-behind_path_vels_[behind_path_vels_pos[human.track_id-1]].path.poses[0].pose.position.x,
                                         human_start.pose.position.y-behind_path_vels_[behind_path_vels_pos[human.track_id-1]].path.poses[0].pose.position.y);

            if(dist_far > 0.5){ // To ensure that the path is recalculated only if the human is deviating from the path
              start_poses_far.push_back(true);
              HumanStartPoseVel human_start_pose_vel = {
                  .id = human.track_id, .pose = human_start, .vel = segment.twist};
              human_start_pose_vels.push_back(human_start_pose_vel);
              behind_path_vels_pos[human.track_id-1] = idx_order;
              behind_path_vels_.clear();
            }
          }
          }
          break;
        }
      }
      idx_order++;
  }
  if (!human_start_pose_vels.empty()) {
    if (transforms_found) {
      for (auto &human_start_pose_vel : human_start_pose_vels) {
        nav_msgs::GetPlan get_plan_srv;

        auto hum_id = human_start_pose_vel.id;
        // get human pose in map frame
        tf::Pose start_pose_tf;
        tf::poseMsgToTF(human_start_pose_vel.pose.pose, start_pose_tf);
        start_pose_tf = human_to_map_tf * start_pose_tf;
        get_plan_srv.request.start.header.frame_id = map_frame_id_;
        get_plan_srv.request.start.header.stamp = now;
        tf::poseTFToMsg(start_pose_tf, get_plan_srv.request.start.pose);

        // calculate human pose behind robot
        if(!check_path){
          check_path = true;
          tf::Transform behind_tr;
          behind_tr.setOrigin(tf::Vector3(-human_dist_behind_robot_, 0.0, 0.0));
          behind_tr.setRotation(
              tf::createQuaternionFromYaw(human_angle_behind_robot_));
          behind_tr = robot_to_map_tf * behind_tr;
          tf::transformTFToMsg(behind_tr, behind_pose);
        }

        get_plan_srv.request.goal.header.frame_id = map_frame_id_;
        get_plan_srv.request.goal.header.stamp = now;
        get_plan_srv.request.goal.pose.position.x = behind_pose.translation.x;
        get_plan_srv.request.goal.pose.position.y = behind_pose.translation.y;
        get_plan_srv.request.goal.pose.position.z = behind_pose.translation.z;
        get_plan_srv.request.goal.pose.orientation = behind_pose.rotation;

        ROS_DEBUG_NAMED(NODE_NAME, "human start: x=%.2f, y=%.2f, theta=%.2f, "
                                   "goal: x=%.2f, y=%.2f, theta=%.2f",
                        get_plan_srv.request.start.pose.position.x,
                        get_plan_srv.request.start.pose.position.y,
                        tf::getYaw(get_plan_srv.request.start.pose.orientation),
                        get_plan_srv.request.goal.pose.position.x,
                        get_plan_srv.request.goal.pose.position.y,
                        tf::getYaw(get_plan_srv.request.goal.pose.orientation));

        // make plan for human
        if (get_plan_client_) {
          if (get_plan_client_.call(get_plan_srv)) {
            if (get_plan_srv.response.plan.poses.size() > 0) {
              HumanPathVel human_path_vel;
              human_path_vel.id = human_start_pose_vel.id;
              human_path_vel.path = get_plan_srv.response.plan;
              human_path_vel.start_vel = human_start_pose_vel.vel;
              behind_path_vels_.push_back(human_path_vel);
              got_new_human_paths_ = true;
            } else {
              ROS_WARN_NAMED(NODE_NAME, "Got empty path for human, start or "
                                        "goal position is probably invalid");
            }
          } else {
            ROS_WARN_NAMED(NODE_NAME, "Failed to call %s service",
                           get_plan_srv_name_.c_str());
          }
        } else {
          ROS_WARN_NAMED(NODE_NAME,
                         "%s service does not exist, re-trying to subscribe",
                         get_plan_srv_name_.c_str());
          ros::NodeHandle private_nh("~/");
          get_plan_client_ = private_nh.serviceClient<nav_msgs::GetPlan>(
              get_plan_srv_name_, true);
        }
      }
    }
  }

  return predictHumansFromPaths(req, res, behind_path_vels_);
}


bool HumanPathPrediction::predictHumansGoal(
    human_path_prediction::HumanPosePredict::Request &req,
    human_path_prediction::HumanPosePredict::Response &res) {
  auto now = ros::Time::now();
  auto tracked_humans = tracked_humans_;

  // get robot pose
  tf::StampedTransform robot_to_map_tf, human_to_map_tf;
  bool transforms_found = false;
  try {
    tf_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0),
                        robot_to_map_tf);
    tf_.lookupTransform(map_frame_id_, tracked_humans.header.frame_id,
                        ros::Time(0), human_to_map_tf);
    transforms_found = true;
  } catch (tf::LookupException &ex) {
    ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n",
                    ex.what());
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
  } catch (tf::ExtrapolationException &ex) {
    ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
  }

  // first check if path calculation is needed, and for whom
  std::vector<HumanStartPoseVel> human_start_pose_vels;
  std::vector<bool> start_poses_far;
  int idx_order = 0;

  for (auto &human : tracked_humans.humans) {
    behind_path_vels_pos.push_back(-1);
    if(std::find(req.ids.begin(),req.ids.end(),human.track_id)==req.ids.end()){
      continue;
    }
    bool path_exist = false;
    for (auto path_vel : behind_path_vels_) {
      if (path_vel.id == human.track_id) {
        path_exist = true;
        break;
      }
    }
    std_srvs::Trigger g_srv;
    goal_change_srv_.call(g_srv);

      // get human pose
      for (auto &segment : human.segments) {
        if (segment.type == default_human_part_) {
          geometry_msgs::PoseStamped human_start;
          human_start.header.frame_id = tracked_humans.header.frame_id;
          human_start.header.stamp = now;
          human_start.pose = segment.pose.pose;

          tf::Pose start_pose_tf;
          geometry_msgs::Pose start_pose_;
          tf::poseMsgToTF(human_start.pose, start_pose_tf);
          start_pose_tf = human_to_map_tf * start_pose_tf;
          tf::poseTFToMsg(start_pose_tf, start_pose_);

          if (!path_exist || g_srv.response.success) {
            HumanStartPoseVel human_start_pose_vel = {
                 .id = human.track_id, .pose = human_start, .vel = segment.twist};
             human_start_pose_vels.push_back(human_start_pose_vel);
             behind_path_vels_pos[human.track_id-1] = idx_order;
          }
          else{
            if(behind_path_vels_.size()>=human.track_id && std::find(req.ids.begin(),req.ids.end(),human.track_id)!=req.ids.end()){
            double dist_far = std::hypot(human_start.pose.position.x-behind_path_vels_[behind_path_vels_pos[human.track_id-1]].path.poses[0].pose.position.x,
                                         human_start.pose.position.y-behind_path_vels_[behind_path_vels_pos[human.track_id-1]].path.poses[0].pose.position.y);

            if(dist_far > 0.5){
              start_poses_far.push_back(true);
              HumanStartPoseVel human_start_pose_vel = {
                  .id = human.track_id, .pose = human_start, .vel = segment.twist};
              human_start_pose_vels.push_back(human_start_pose_vel);
              behind_path_vels_pos[human.track_id-1] = idx_order;
              behind_path_vels_.clear();
            }
           }
          }
          break;
        }
      }
      idx_order++;
  }

  if (!human_start_pose_vels.empty()) {
    if (transforms_found) {
      for (auto &human_start_pose_vel : human_start_pose_vels) {
        nav_msgs::GetPlan get_plan_srv;

        // get human pose in map frame
        tf::Pose start_pose_tf;
        tf::poseMsgToTF(human_start_pose_vel.pose.pose, start_pose_tf);
        start_pose_tf = human_to_map_tf * start_pose_tf;
        get_plan_srv.request.start.header.frame_id = map_frame_id_;
        get_plan_srv.request.start.header.stamp = now;
        tf::poseTFToMsg(start_pose_tf, get_plan_srv.request.start.pose);

        get_plan_srv.request.goal.header.frame_id = map_frame_id_;
        get_plan_srv.request.goal.header.stamp = now;
        get_plan_srv.request.goal.pose.position.x = predicted_goal_->goal.pose.position.x;
        get_plan_srv.request.goal.pose.position.y = predicted_goal_->goal.pose.position.y;
        get_plan_srv.request.goal.pose.position.z = predicted_goal_->goal.pose.position.z;
        get_plan_srv.request.goal.pose.orientation = predicted_goal_->goal.pose.orientation;

        ROS_DEBUG_NAMED(NODE_NAME, "human start: x=%.2f, y=%.2f, theta=%.2f, "
                                   "goal: x=%.2f, y=%.2f, theta=%.2f",
                        get_plan_srv.request.start.pose.position.x,
                        get_plan_srv.request.start.pose.position.y,
                        tf::getYaw(get_plan_srv.request.start.pose.orientation),
                        get_plan_srv.request.goal.pose.position.x,
                        get_plan_srv.request.goal.pose.position.y,
                        tf::getYaw(get_plan_srv.request.goal.pose.orientation));

        // make plan for human
        if (get_plan_client_) {
          if (get_plan_client_.call(get_plan_srv)) {
            if (get_plan_srv.response.plan.poses.size() > 0) {
              HumanPathVel human_path_vel;
              human_path_vel.id = human_start_pose_vel.id;
              human_path_vel.path = get_plan_srv.response.plan;
              human_path_vel.start_vel = human_start_pose_vel.vel;
              behind_path_vels_.push_back(human_path_vel);
              got_new_human_paths_ = true;
            } else {
              ROS_WARN_NAMED(NODE_NAME, "Got empty path for human, start or "
                                        "goal position is probably invalid");
            }
          } else {
            ROS_WARN_NAMED(NODE_NAME, "Failed to call %s service",
                           get_plan_srv_name_.c_str());
          }
        } else {
          ROS_WARN_NAMED(NODE_NAME,
                         "%s service does not exist, re-trying to subscribe",
                         get_plan_srv_name_.c_str());
          ros::NodeHandle private_nh("~/");
          get_plan_client_ = private_nh.serviceClient<nav_msgs::GetPlan>(
              get_plan_srv_name_, true);
        }
      }
    }
  }

  return predictHumansFromPaths(req, res, behind_path_vels_);
}


bool HumanPathPrediction::predictHumansFromPaths(
    human_path_prediction::HumanPosePredict::Request &req,
    human_path_prediction::HumanPosePredict::Response &res,
    const std::vector<HumanPathVel> &path_vels) {
  auto tracked_humans = tracked_humans_;

  if (got_new_human_paths_) {
    for (auto human_path_vel : path_vels) {
      auto &poses = human_path_vel.path.poses;
      if (!poses.empty()) {
        human_path_prediction::PredictedPoses predicted_poses;
        predicted_poses.id = human_path_vel.id;

        auto lin_vel = std::hypot(human_path_vel.start_vel.twist.linear.x,
                                  human_path_vel.start_vel.twist.linear.y);
        auto now = ros::Time::now();

        predicted_poses.poses.resize(poses.size());
        for (size_t i = 0; i < poses.size(); ++i) {
          auto &pose = poses[i];
          geometry_msgs::PoseWithCovarianceStamped predicted_pose;
          if (i == 0 || lin_vel == 0.0) {
            predicted_pose.header.stamp = now;
          } else {
            auto &last_pose = poses[i - 1];
            auto dist =
                std::hypot(pose.pose.position.x - last_pose.pose.position.x,
                           pose.pose.position.y - last_pose.pose.position.y);
            predicted_pose.header.stamp =
                predicted_poses.poses[i - 1].header.stamp +
                ros::Duration(dist / lin_vel);
          }
          predicted_pose.header.frame_id = pose.header.frame_id;
          predicted_pose.pose.pose = pose.pose;
          predicted_poses.poses[i] = predicted_pose;
        }

        for (auto it = last_predicted_poses_.begin();
             it != last_predicted_poses_.end(); ++it) {
          if (it->id == predicted_poses.id) {
            last_predicted_poses_.erase(it);
            break;
          }
        }
        last_predicted_poses_.push_back(predicted_poses);

        last_prune_indices_.erase(predicted_poses.id);

        for (auto it = tracked_humans.humans.begin();
             it != tracked_humans.humans.end(); ++it) {
          if (it->track_id == predicted_poses.id) {
            tracked_humans.humans.erase(it);
            break;
          }
        }
        ROS_DEBUG_NAMED(
            NODE_NAME,
            "Processed new path for human %ld with %ld poses in frame %s",
            human_path_vel.id, predicted_poses.poses.size(),
            predicted_poses.poses.front().header.frame_id.c_str());
      }
    }
  }
  got_new_human_paths_ = false;

  for (auto &poses : last_predicted_poses_) {
    if (!poses.poses.empty()) {
      geometry_msgs::PoseStamped start_pose;
      geometry_msgs::TwistStamped start_twist;
      if (transformPoseTwist(tracked_humans, poses.id,
                             poses.poses.front().header.frame_id, start_pose,
                             start_twist)) {
        auto last_prune_index_it = last_prune_indices_.find(poses.id);
        auto begin_index = (last_prune_index_it != last_prune_indices_.end())
                               ? last_prune_index_it->second
                               : 0;
        auto prune_index = prunePath(begin_index, start_pose.pose, poses.poses);
        last_prune_indices_[poses.id] = prune_index;
        if (prune_index < 0 || prune_index > poses.poses.size()) {
          ROS_ERROR_NAMED(NODE_NAME, "Logical error, cannot prune path");
          continue;
        }

        // std::vector<geometry_msgs::PoseWithCovarianceStamped> pruned_path(
        //     poses.poses.begin() + prune_index, poses.poses.end());
        geometry_msgs::PoseWithCovarianceStamped start_pose_co;
        start_pose_co.header.stamp = start_pose.header.stamp;
        start_pose_co.header.frame_id = start_pose.header.frame_id;
        start_pose_co.pose.pose = start_pose.pose;
        std::vector<geometry_msgs::PoseWithCovarianceStamped> pruned_path;
        pruned_path.push_back(start_pose_co);
        pruned_path.insert(pruned_path.end(), poses.poses.begin() + prune_index,
                           poses.poses.end());

        if (!pruned_path.empty()) {

          // update time stamps for the predicted path
          auto lin_vel = std::hypot(start_twist.twist.linear.x,
                                    start_twist.twist.linear.y);
          auto now = ros::Time::now();
          for (size_t i = 0; i < pruned_path.size(); i++) {
            if (i == 0 || lin_vel == 0) {
              pruned_path[i].header.stamp = now;
            } else {
              auto &pose = pruned_path[i].pose.pose;
              auto &last_pose = pruned_path[i - 1].pose.pose;
              auto dist = std::hypot(pose.position.x - last_pose.position.x,
                                     pose.position.y - last_pose.position.y);
              pruned_path[i].header.stamp = pruned_path[i - 1].header.stamp +
                                            ros::Duration(dist / lin_vel);
            }
          }

          human_path_prediction::PredictedPoses predicted_poses;
          predicted_poses.id = poses.id;
          predicted_poses.start_velocity = start_twist;
          predicted_poses.poses = pruned_path;

          res.predicted_humans_poses.push_back(predicted_poses);
          //ROS_INFO("Pushed the poses");
          ROS_DEBUG_NAMED(
              NODE_NAME,
              "Giving path of %ld points from %ld points for human %ld",
              predicted_poses.poses.size(), poses.poses.size(), poses.id);
        }
      }
    }
  }

  return true;
}

bool HumanPathPrediction::setPublishMarkers(std_srvs::SetBool::Request &req,
                                            std_srvs::SetBool::Response &res) {
  publish_markers_ = req.data;
  res.success = true;
  res.message = "Prediction markers publishing " + publish_markers_
                    ? "enabled"
                    : "disabled";
  return true;
}

bool HumanPathPrediction::setExternalGoal(human_path_prediction::HumanGoal::Request &req, human_path_prediction::HumanGoal::Response &res){
  ROS_DEBUG_NAMED(NODE_NAME, "Received new human goal");
  got_external_goal = true;
  external_goals_.clear();
  behind_path_vels_.clear();
  for(auto & goal: req.goals){
    external_goals_.push_back(goal);
  }

  res.success=true;
  res.message="Goal has been set.";
}

bool HumanPathPrediction::checkExternalGoal(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	res.message = got_external_goal ? "Goal Set":"Goal not Set";
	res.success = got_external_goal;
	return true;
}

bool HumanPathPrediction::transformPoseTwist(
    const human_msgs::TrackedHumans &tracked_humans, const uint64_t &human_id,
    const std::string &to_frame, geometry_msgs::PoseStamped &pose,
    geometry_msgs::TwistStamped &twist) {
  for (auto &human : tracked_humans.humans) {
    if (human.track_id == human_id) {
      for (auto &segment : human.segments) {
        if (segment.type == default_human_part_) {
          geometry_msgs::PoseStamped pose_ut;
          pose_ut.header.stamp = tracked_humans.header.stamp;
          pose_ut.header.frame_id = tracked_humans.header.frame_id;
          pose_ut.pose = segment.pose.pose;
          twist.header.stamp = tracked_humans.header.stamp;
          twist.header.frame_id = tracked_humans.header.frame_id;
          twist.twist = segment.twist.twist;
          try {
            tf::Stamped<tf::Pose> pose_tf;
            tf::poseStampedMsgToTF(pose_ut, pose_tf);
            tf::StampedTransform start_pose_to_plan_transform;
            tf_.waitForTransform(to_frame, pose_ut.header.frame_id,
                                 ros::Time(0), ros::Duration(0.5));
            tf_.lookupTransform(to_frame, pose_ut.header.frame_id, ros::Time(0),
                                start_pose_to_plan_transform);
            pose_tf.setData(start_pose_to_plan_transform * pose_tf);
            pose_tf.frame_id_ = to_frame;
            tf::poseStampedTFToMsg(pose_tf, pose);

            geometry_msgs::Twist start_twist_to_plan_transform;
            tf_.lookupTwist(to_frame, twist.header.frame_id, ros::Time(0),
                            ros::Duration(0.1), start_twist_to_plan_transform);
            twist.twist.linear.x -= start_twist_to_plan_transform.linear.x;
            twist.twist.linear.y -= start_twist_to_plan_transform.linear.y;
            twist.twist.angular.z -= start_twist_to_plan_transform.angular.z;
            twist.header.frame_id = to_frame;
            return true;
          } catch (tf::LookupException &ex) {
            ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n",
                            ex.what());
          } catch (tf::ConnectivityException &ex) {
            ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
          } catch (tf::ExtrapolationException &ex) {
            ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
          }
          break;
        }
      }
      break;
    }
  }
  return false;
}

size_t HumanPathPrediction::prunePath(
    size_t begin_index, const geometry_msgs::Pose &pose,
    const std::vector<geometry_msgs::PoseWithCovarianceStamped> &path) {
  size_t prune_index = begin_index;
  double x_diff, y_diff, sq_diff,
      smallest_sq_diff = std::numeric_limits<double>::max();
  while (begin_index < path.size()) {
    x_diff = path[begin_index].pose.pose.position.x - pose.position.x;
    y_diff = path[begin_index].pose.pose.position.y - pose.position.y;
    sq_diff = x_diff * x_diff + y_diff * y_diff;
    if (sq_diff < smallest_sq_diff) {
      prune_index = begin_index;
      smallest_sq_diff = sq_diff;
    }
    ++begin_index;
  }
  return prune_index;
}

bool HumanPathPrediction::resetExtPaths(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res) {
  got_new_human_paths_ = false;
  got_external_goal = false;
  last_predicted_poses_.clear();
  behind_path_vels_.clear();
  check_path = false;
  return true;
}


}

// handler for something to do before killing the node
void sigintHandler(int sig) {
  ROS_DEBUG_NAMED(NODE_NAME, "node %s will now shutdown", NODE_NAME);

  // the default sigint handler, it calls shutdown() on node
  ros::shutdown();
}

// the main method starts a rosnode and initializes the optotrack_person class
int main(int argc, char **argv) {
  // starting the optotrack_person node
  ros::init(argc, argv, NODE_NAME);
  ROS_DEBUG_NAMED(NODE_NAME, "started %s node", NODE_NAME);

  // initiazling human_pathHeadBehavior class
  human_path_prediction::HumanPathPrediction HumanPathPrediction;
  HumanPathPrediction.initialize();

  // look for sigint and start spinning the node
  signal(SIGINT, sigintHandler);
  ros::spin();

  return 0;
}
