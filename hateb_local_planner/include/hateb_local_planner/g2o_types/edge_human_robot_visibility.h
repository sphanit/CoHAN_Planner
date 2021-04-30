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

#ifndef EDGE_HUMAN_ROBOT_VISIBILITY_H_
#define EDGE_HUMAN_ROBOT_VISIBILITY_H_

#include <hateb_local_planner/g2o_types/vertex_pose.h>
#include <hateb_local_planner/g2o_types/penalties.h>
#include <hateb_local_planner/hateb_config.h>
#include <hateb_local_planner/g2o_types/base_teb_edges.h>

// #include "g2o/core/base_unary_edge.h"

namespace hateb_local_planner {

class EdgeHumanRobotVisibility : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose> {
public:
  EdgeHumanRobotVisibility()
  {
      this->setMeasurement(0.0);
  }

  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setParameters() on EdgeHumanRobotVisibility()");
    const VertexPose *robot_bandpt = static_cast<const VertexPose *>(_vertices[0]);
    const VertexPose *human_bandpt = static_cast<const VertexPose *>(_vertices[1]);
    Eigen::Vector2d d_rtoh = human_bandpt->position() - robot_bandpt->position();
    Eigen::Vector2d d_htor = robot_bandpt->position() - human_bandpt->position();
    Eigen::Vector2d humanLookAt = {cos(human_bandpt->theta()), sin(human_bandpt->theta())};
    Eigen::Vector2d robotLookAt = {cos(robot_bandpt->theta()), sin(robot_bandpt->theta())};
    double deltaPsi = fabs(acos(humanLookAt.dot(d_htor) / (humanLookAt.norm() * d_htor.norm())));
    //ROS_DEBUG_THROTTLE(0.5, "robot_human_angle deg : %f", deltaPsi * 180 / M_PI);
    double c_visibility = 0.0;
    double ang = humanLookAt.dot(robotLookAt);
    // std::cout << "ang " <<ang<< '\n';

    if (deltaPsi >= cfg_->human.fov * M_PI / 180){
      // c_visibility = deltaPsi * ((cos(d_rtoh.x()) + 1) * (cos(d_rtoh.y()) + 1));
      if(ang >=0)
        c_visibility = 5*((std::pow(2,-(std::pow(d_rtoh.x(),2)))) * (std::pow(2,-(std::pow(d_rtoh.y(),2)))));
      else
        c_visibility = 0.;
    }

    _error[0] = penaltyBoundFromAbove(c_visibility, cfg_->hateb.visibility_cost_threshold,
                                      cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeHumanRobotVisibility::computeError() _error[0]=%f\n", _error[0]);
  }

  void setParameters(const HATebConfig &cfg)
  {
      cfg_ = &cfg;
  }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}; // end namespace

#endif
