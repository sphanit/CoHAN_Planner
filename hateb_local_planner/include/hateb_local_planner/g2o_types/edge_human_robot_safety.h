/*/
 * Copyright (c) 2020 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Phani Teja Singamaneni
 */

#ifndef EDGE_HUMAN_ROBOT_SAFETY_H_
#define EDGE_HUMAN_ROBOT_SAFETY_H_

#include <hateb_local_planner/obstacles.h>
#include <hateb_local_planner/robot_footprint_model.h>
#include <hateb_local_planner/g2o_types/vertex_pose.h>
#include <hateb_local_planner/g2o_types/penalties.h>
#include <hateb_local_planner/hateb_config.h>
#include <hateb_local_planner/g2o_types/base_teb_edges.h>

// #include "g2o/core/base_unary_edge.h"

namespace hateb_local_planner {

class EdgeHumanRobotSafety : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose> {
public:
  EdgeHumanRobotSafety()
  {
    this->setMeasurement(0.);
  }

  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && robot_model_ && human_radius_ < std::numeric_limits<double>::infinity(), "You must call setParameters() on EdgeHumanRobotSafety()");
    const VertexPose *robot_bandpt = static_cast<const VertexPose *>(_vertices[0]);
    const VertexPose *human_bandpt = static_cast<const VertexPose *>(_vertices[1]);
    static_cast<PointObstacle *>(obs_)->setCentroid(human_bandpt->x(), human_bandpt->y());

    double dist = robot_model_->calculateDistance(robot_bandpt->pose(), obs_) - human_radius_;
    // std::cout << "robot_bandpt->pose() "<<robot_bandpt->pose() << '\n';

    ROS_DEBUG_THROTTLE(0.5, "human robot dist = %f", dist);

    // _error[0] = penaltyBoundFromBelow(dist, min_dist_ , cfg_->optim.penalty_epsilon);
    _error[0] = penaltyBoundFromBelowQuad(dist, min_dist_ , cfg_->optim.penalty_epsilon);


    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeHumanRobotSafety::computeError() _error[0]=%f\n", _error[0]);
  }

  void setRobotModel(const BaseRobotFootprintModel *robot_model) {
    robot_model_ = robot_model;
  }

  void setHumanRadius(const double human_radius) {
    human_radius_ = human_radius;
  }

  void setParameters(const HATebConfig &cfg, const BaseRobotFootprintModel *robot_model, const double human_radius, const double min_dist) {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    human_radius_ = human_radius;
    min_dist_ = min_dist;
  }

protected:
  const BaseRobotFootprintModel *robot_model_;
  Obstacle *obs_ = new PointObstacle();
  double human_radius_ = std::numeric_limits<double>::infinity();
  double min_dist_ = 0.0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}; // end namespace

#endif
