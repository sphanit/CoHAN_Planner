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

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

// teb stuff
#include <hateb_local_planner/TrajectoryMsg.h>
#include <hateb_local_planner/robot_footprint_model.h>
#include <hateb_local_planner/hateb_config.h>
#include <hateb_local_planner/timed_elastic_band.h>

// ros stuff
#include <base_local_planner/goal_functions.h>
#include <ros/publisher.h>
#include <tf/transform_listener.h>

// boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

// std
#include <iterator>

// messages
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <human_msgs/HumanPathArray.h>
#include <human_msgs/HumanTimeToGoal.h>
#include <human_msgs/HumanTimeToGoalArray.h>
#include <human_msgs/HumanTrajectoryArray.h>
#include <human_msgs/TrackedHumans.h>
#include <human_msgs/TrackedSegmentType.h>
#include <human_msgs/TimeToGoal.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace hateb_local_planner {

typedef struct {
  std::vector<geometry_msgs::PoseStamped> plan_before;
  std::vector<TrajectoryPointMsg> optimized_trajectory;
  std::vector<geometry_msgs::PoseStamped> plan_after;
} PlanTrajCombined;

typedef struct {
  std::vector<geometry_msgs::PoseStamped> plan_before;
  std::vector<geometry_msgs::PoseStamped> plan_to_optimize;
  std::vector<geometry_msgs::PoseStamped> plan_after;
} PlanCombined;

typedef struct {
  uint64_t id;
  std::vector<geometry_msgs::PoseStamped> plan_before;
  std::vector<TrajectoryPointMsg> optimized_trajectory;
  std::vector<geometry_msgs::PoseStamped> plan_after;
} HumanPlanTrajCombined;

typedef struct {
  uint64_t id;
  std::vector<geometry_msgs::PoseStamped> plan_before;
  std::vector<geometry_msgs::PoseStamped> plan_to_optimize;
  std::vector<geometry_msgs::PoseStamped> plan_after;
} HumanPlanCombined;

class TebOptimalPlanner; //!< Forward Declaration

/**
 * @class TebVisualization
 * @brief Visualize stuff from the hateb_local_planner
 */
class TebVisualization
{
public:
  /**
   * @brief Default constructor
   * @remarks do not forget to call initialize()
   */
  TebVisualization();

  /**
   * @brief Constructor that initializes the class and registers topics
   * @param nh local ros::NodeHandle
   * @param cfg const reference to the HATebConfig class for parameters
   */
  TebVisualization(ros::NodeHandle& nh, const HATebConfig& cfg);

  /**
   * @brief Initializes the class and registers topics.
   *
   * Call this function if only the default constructor has been called before.
   * @param nh local ros::NodeHandle
   * @param cfg const reference to the HATebConfig class for parameters
   */
  void initialize(ros::NodeHandle& nh, const HATebConfig& cfg);

  /** @name Publish to topics */
  //@{

  /**
   * @brief Publish a given global plan to the ros topic \e ../../global_plan
   * @param global_plan Pose array describing the global plan
   */
  void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) const;
  void publishHumanGlobalPlans(const std::vector<HumanPlanCombined> &humans_plans) const;

  /**
   * @brief Publish a given local plan to the ros topic \e ../../local_plan
   * @param local_plan Pose array describing the local plan
   */
  void publishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& local_plan) const;


  void publishTrackedHumans(const human_msgs::TrackedHumansConstPtr &humans);

  /**
   * @brief Publish Timed_Elastic_Band related stuff (local plan, pose sequence).
   *
   * Given a Timed_Elastic_Band instance, publish the local plan to  \e ../../local_plan
   * and the pose sequence to  \e ../../teb_poses.
   * @param teb const reference to a Timed_Elastic_Band
   */
  void publishLocalPlanAndPoses(const TimedElasticBand &teb, const BaseRobotFootprintModel &robot_model, const double fp_size, const std_msgs::ColorRGBA &color = toColorMsg(0.5, 0.0, 0.8, 0.0));
  void publishHumanLocalPlansAndPoses(const std::map<uint64_t, TimedElasticBand> &humans_tebs_map, const BaseRobotFootprintModel &human_model, const double fp_size, const std_msgs::ColorRGBA &color= toColorMsg(0.5, 0.0, 0.8, 0.0));

  void publishTrajectory(const PlanTrajCombined &plan_traj_combined);
  void publishHumanTrajectories(const std::vector<HumanPlanTrajCombined> &humans_plans_combined);

  /**
   * @brief Publish the visualization of the robot model
   *
   * @param current_pose Current pose of the robot
   * @param robot_model Subclass of BaseRobotFootprintModel
   * @param ns Namespace for the marker objects
   * @param color Color of the footprint
   */
  void publishRobotFootprintModel(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model, const std::string& ns = "RobotFootprintModel",
                                  const std_msgs::ColorRGBA& color = toColorMsg(0.5, 0.0, 0.8, 0.0));

  /**
   * @brief Publish the robot footprints related to infeasible poses
   *
   * @param current_pose Current pose of the robot
   * @param robot_model Subclass of BaseRobotFootprintModel
   */
  void publishInfeasibleRobotPose(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model);

  /**
   * @brief Publish obstacle positions to the ros topic \e ../../teb_markers
   * @todo Move filling of the marker message to polygon class in order to avoid checking types.
   * @param obstacles Obstacle container
   */
  void publishObstacles(const ObstContainer& obstacles) const;

  /**
   * @brief Publish via-points to the ros topic \e ../../teb_markers
   * @param via_points via-point container
   */
  void publishViaPoints(const std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns = "ViaPoints") const;

  /**
   * @brief Publish a boost::adjacency_list (boost's graph datatype) via markers.
   * @remarks Make sure that vertices of the graph contain a member \c pos as \c Eigen::Vector2d type
   *	      to query metric position values.
   * @param graph Const reference to the boost::adjacency_list (graph)
   * @param ns_prefix Namespace prefix for the marker objects (the strings "Edges" and "Vertices" will be appended)
   * @tparam GraphType boost::graph object in which vertices has the field/member \c pos.
   */
  template <typename GraphType>
  void publishGraph(const GraphType& graph, const std::string& ns_prefix = "Graph");

  /**
   * @brief Publish multiple 2D paths (each path given as a point sequence) from a container class.
   *
   * Provide a std::vector< std::vector< T > > in which T.x() and T.y() exist
   * and std::vector could be individually substituded by std::list / std::deque /...
   *
   * A common point-type for object T could be Eigen::Vector2d.
   *
   * T could be also a raw pointer std::vector< std::vector< T* > >.
   *
   * @code
   * 	typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > PathType; // could be a list or deque as well ...
   *    std::vector<PathType> path_container(2); // init 2 empty paths; the container could be a list or deque as well ...
   * 	// Fill path_container.at(0) with Eigen::Vector2d elements, we skip that here
   * 	// Fill path_container.at(1) with Eigen::Vector2d elements, we skip that here
   *    publishPathContainer( path_container.begin(), path_container.end() );
   * @endcode
   *
   * @remarks Actually the underlying path does not necessarily need to be a Eigen::Vector2d sequence.
   *          Eigen::Vector2d can be replaced with any datatype that implement public x() and y() methods.\n
   * @param first Bidirectional iterator pointing to the begin of the path
   * @param last Bidirectional iterator pointing to the end of the path
   * @param ns Namespace for the marker objects (the strings "Edges" and "Vertices" will be appended)
   * @tparam BidirIter Bidirectional iterator to a 2D path (sequence of Eigen::Vector2d elements) in a container
   */
  template <typename BidirIter>
  void publishPathContainer(BidirIter first, BidirIter last, const std::string& ns = "PathContainer");

  /**
   * @brief Publish multiple Tebs from a container class (publish as marker message).
   *
   * @param teb_planner Container of boost::shared_ptr< TebOptPlannerPtr >
   * @param ns Namespace for the marker objects
   */
  void publishTebContainer(const std::vector< boost::shared_ptr<TebOptimalPlanner> >& teb_planner, const std::string& ns = "TebContainer");

  /**
   * @brief Publish a feedback message (multiple trajectory version)
   *
   * The feedback message contains the all planned trajectory candidates (e.g. if planning in distinctive topologies is turned on).
   * Each trajectory is composed of the sequence of poses, the velocity profile and temporal information.
   * The feedback message also contains a list of active obstacles.
   * @param teb_planners container with multiple tebs (resp. their planner instances)
   * @param selected_trajectory_idx Idx of the currently selected trajectory in \c teb_planners
   * @param obstacles Container of obstacles
   */
  void publishFeedbackMessage(const std::vector< boost::shared_ptr<TebOptimalPlanner> >& teb_planners, unsigned int selected_trajectory_idx, const ObstContainer& obstacles);

  /**
   * @brief Publish a feedback message (single trajectory overload)
   *
   * The feedback message contains the planned trajectory
   * that is composed of the sequence of poses, the velocity profile and temporal information.
   * The feedback message also contains a list of active obstacles.
   * @param teb_planner the planning instance
   * @param obstacles Container of obstacles
   */
  void publishFeedbackMessage(const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles);

  //@}

  /**
   * @brief Helper function to generate a color message from single values
   * @param a Alpha value
   * @param r Red value
   * @param g Green value
   * @param b Blue value
   * @return Color message
   */
  static std_msgs::ColorRGBA toColorMsg(double a, double r, double g, double b);

  void setMarkerColour(visualization_msgs::Marker &marker, double itr, double n);
  void publishMode(int Mode);

protected:

  /**
   * @brief Small helper function that checks if initialize() has been called and prints an error message if not.
   * @return \c true if not initialized, \c false if everything is ok
   */
  bool printErrorWhenNotInitialized() const;

  ros::Publisher global_plan_pub_; //!< Publisher for the global plan
  ros::Publisher local_plan_pub_;  //!< Publisher for the local plan
  ros::Publisher local_traj_pub_;
  ros::Publisher humans_global_plans_pub_; //!< Publisher for the local plan
  ros::Publisher humans_local_plans_pub_;  //!< Publisher for the local plan
  ros::Publisher humans_local_trajs_pub_;
  ros::Publisher teb_poses_pub_, teb_fp_poses_pub_; //!< Publisher for the trajectory pose sequence
  ros::Publisher humans_tebs_poses_pub_, humans_tebs_fp_poses_pub_; //!< Publisher for the trajectory pose sequence
  ros::Publisher teb_marker_pub_; //!< Publisher for visualization markers
  ros::Publisher feedback_pub_, mode_text_pub;   //!< Publisher for the feedback message and mode for analysis and debug purposes
  ros::Publisher robot_traj_time_pub_, robot_path_time_pub_;
  ros::Publisher robot_next_pose_pub_, human_next_pose_pub_; // Pulishers for pose tracking
  ros::Publisher human_trajs_time_pub_, human_paths_time_pub_;
  ros::Publisher human_marker_pub ,human_arrow_pub;
  ros::Subscriber tracked_humans_sub_;
  std::vector<double> vel_robot, vel_human;
  tf::TransformListener tf_;


  const HATebConfig* cfg_; //!< Config class that stores and manages all related parameters

  bool initialized_; //!< Keeps track about the correct initialization of this class
  ros::Timer clearing_timer_;
  void clearingTimerCB(const ros::TimerEvent &event);
  bool last_publish_robot_global_plan, last_publish_robot_local_plan,
      last_publish_robot_local_plan_poses,
      last_publish_robot_local_plan_fp_poses, last_publish_human_global_plans,
      last_publish_human_local_plans, last_publish_human_local_plan_poses,
      last_publish_human_local_plan_fp_poses;

  mutable int last_robot_fp_poses_idx_, last_human_fp_poses_idx_;


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared instances of the TebVisualization
typedef boost::shared_ptr<TebVisualization> TebVisualizationPtr;

//! Abbrev. for shared instances of the TebVisualization (read-only)
typedef boost::shared_ptr<const TebVisualization> TebVisualizationConstPtr;


} // namespace hateb_local_planner


// Include template method implementations / definitions
#include <hateb_local_planner/visualization.hpp>

#endif /* VISUALIZATION_H_ */
