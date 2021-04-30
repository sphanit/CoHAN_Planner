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

#ifndef HUMAN_PATH_PREDICTION_H_
#define HUMAN_PATH_PREDICTION_H_

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <human_path_prediction/HumanPathPredictionConfig.h>

#include <human_msgs/TrackedHumans.h>
#include <human_msgs/TrackedSegmentType.h>
#include <human_msgs/HumanPathArray.h>
#include <human_msgs/HumanTrajectory.h>
#include <human_msgs/HumanTrajectoryArray.h>
#include <human_path_prediction/HumanPosePredict.h>
#include <human_path_prediction/PredictedGoal.h>
#include <human_path_prediction/HumanGoal.h>
#include <human_path_prediction/HumanPose.h>
#include <tf/transform_listener.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>

#include <visualization_msgs/MarkerArray.h>

namespace human_path_prediction {
class HumanPathPrediction {
public:
  HumanPathPrediction();
  ~HumanPathPrediction();

  void initialize();

  void setParams(std::vector<double> velscale_scales, double velscale_angle,
                 double velscale_mul, double velobs_mul, double velobs_min_rad,
                 double velobs_max_rad, double velobs_max_rad_time,
                 bool velobs_use_ang);

private:
  // ros subscribers and publishers
  ros::Subscriber tracked_humans_sub_, external_paths_sub_, external_trajs_sub_,robot_pos_sub_, predicted_goal_sub_;
  ros::Publisher predicted_humans_pub_;
  bool done_cfg;

  // ros services
  ros::ServiceServer predict_humans_server_, reset_ext_paths_server_,
      publish_markers_srv_, set_goal_srv_, set_goal_call_srv_;
  ros::ServiceClient get_plan_client_,goal_change_srv_;

  // dynamic reconfigure variables
  dynamic_reconfigure::Server<HumanPathPredictionConfig> *dsrv_;
  void reconfigureCB(HumanPathPredictionConfig &config, uint32_t level);

  // subscriber callbacks
  void trackedHumansCB(const human_msgs::TrackedHumans &tracked_humans);
  void
  externalPathsCB(const human_msgs::HumanPathArray::ConstPtr &external_paths);

  tf::TransformListener tf_;

  std::string tracked_humans_sub_topic_, external_paths_sub_topic_,
      reset_ext_paths_service_name_, predict_service_name_,
      predicted_humans_markers_pub_topic_, publish_markers_srv_name_,
      get_plan_srv_name_;
  int default_human_part_;
  bool publish_markers_, showing_markers_, got_new_human_paths_, got_new_goal, got_external_goal;
  std::string robot_frame_id_, map_frame_id_;
  double human_dist_behind_robot_, human_angle_behind_robot_;

  struct HumanPathVel {
    uint64_t id;
    nav_msgs::Path path;
    geometry_msgs::TwistWithCovariance start_vel;
  };

  struct HumanTrajVel {
    uint64_t id;
    human_msgs::TrajectoryPoint traj;
    geometry_msgs::TwistWithCovariance start_vel;
  };

  struct HumanStartPoseVel {
    uint64_t id;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistWithCovariance vel;
  };

  human_msgs::TrackedHumans tracked_humans_;
  human_msgs::HumanPathArray::ConstPtr external_paths_;
  human_path_prediction::PredictedGoal::ConstPtr predicted_goal_;
  human_msgs::HumanPathArray external_paths2_;
  human_msgs::HumanTrajectoryArrayConstPtr external_trajs_;
  std::vector<human_path_prediction::HumanPose> external_goals_;

  std::vector<HumanPathVel> behind_path_vels_;
  std::vector<int> behind_path_vels_pos;
  std::vector<human_path_prediction::PredictedPoses> last_predicted_poses_;
  std::map<uint64_t, size_t> last_prune_indices_;
  std::vector<double> velscale_scales_;
  std::map<uint64_t, int> last_markers_size_map;
  double velscale_angle_, velscale_mul_, velobs_mul_, velobs_min_rad_,
      velobs_max_rad_, velobs_max_rad_time_;
  bool velobs_use_ang_;
  visualization_msgs::MarkerArray predicted_humans_markers_;
  geometry_msgs::Transform behind_pose;
  bool check_path;

  bool predictHumans(human_path_prediction::HumanPosePredict::Request &req,
                     human_path_prediction::HumanPosePredict::Response &res);
  bool predictHumansVelScale(human_path_prediction::HumanPosePredict::Request &req,
                             human_path_prediction::HumanPosePredict::Response &res);
  bool predictHumansVelObs(human_path_prediction::HumanPosePredict::Request &req,
                           human_path_prediction::HumanPosePredict::Response &res);
  bool predictHumansExternal(human_path_prediction::HumanPosePredict::Request &req,
                             human_path_prediction::HumanPosePredict::Response &res);
  bool predictHumansBehind(human_path_prediction::HumanPosePredict::Request &req,
                           human_path_prediction::HumanPosePredict::Response &res);
  bool predictHumansGoal(human_path_prediction::HumanPosePredict::Request &req,
			   human_path_prediction::HumanPosePredict::Response &res);
  bool predictHumansFromPaths(human_path_prediction::HumanPosePredict::Request &req,
                              human_path_prediction::HumanPosePredict::Response &res,
                              const std::vector<HumanPathVel> &path_vels);
  bool setPublishMarkers(std_srvs::SetBool::Request &req,
                         std_srvs::SetBool::Response &res);
  bool setExternalGoal(human_path_prediction::HumanGoal::Request &req, human_path_prediction::HumanGoal::Response &res);
  bool checkExternalGoal(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool transformPoseTwist(const human_msgs::TrackedHumans &tracked_humans,
                          const uint64_t &human_id, const std::string &to_frame,
                          geometry_msgs::PoseStamped &pose,
                          geometry_msgs::TwistStamped &twist);
  void externalTrajsCB(const human_msgs::HumanTrajectoryArrayConstPtr &traj_array);

  void predictedGoalCB(const human_path_prediction::PredictedGoal::ConstPtr& predicted_goal);

  double checkdist(geometry_msgs::Pose human, geometry_msgs::Pose robot);


  size_t
  prunePath(size_t begin_index, const geometry_msgs::Pose &pose,
            const std::vector<geometry_msgs::PoseWithCovarianceStamped> &path);

  bool resetExtPaths(std_srvs::Empty::Request &req,
                     std_srvs::Empty::Response &res);
};
}

#endif // HUMAN_PATH_PREDICTION_H_
