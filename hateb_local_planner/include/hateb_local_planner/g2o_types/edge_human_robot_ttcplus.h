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

#ifndef EDGE_HUMAN_ROBOT_TTCplus_H_
#define EDGE_HUMAN_ROBOT_TTCplus_H_


#include <hateb_local_planner/g2o_types/vertex_pose.h>
#include <hateb_local_planner/g2o_types/vertex_timediff.h>
#include <hateb_local_planner/g2o_types/penalties.h>
#include <hateb_local_planner/hateb_config.h>
#include <std_msgs/Header.h>
#include "sstream"
#include <hateb_local_planner/g2o_types/base_teb_edges.h>

// #include "g2o/core/base_multi_edge.h"

namespace hateb_local_planner {

 class EdgeHumanRobotTTCplus : public BaseTebMultiEdge<1, double> {
public:
  EdgeHumanRobotTTCplus() {
    this->resize(6);
  }


   void computeError() {
    ROS_ASSERT_MSG(cfg_ && (radius_sum_ < std::numeric_limits<double>::infinity()), "You must call setParameters() on EdgeHumanRobotTTCplus()");
    const VertexPose *robot_bandpt = static_cast<const VertexPose *>(_vertices[0]);
    const VertexPose *robot_bandpt_nxt = static_cast<const VertexPose *>(_vertices[1]);
    const VertexTimeDiff *dt_robot = static_cast<const VertexTimeDiff *>(_vertices[2]);
    const VertexPose *human_bandpt = static_cast<const VertexPose *>(_vertices[3]);
    const VertexPose *human_bandpt_nxt = static_cast<const VertexPose *>(_vertices[4]);
    const VertexTimeDiff *dt_human = static_cast<const VertexTimeDiff *>(_vertices[5]);

    Eigen::Vector2d diff_robot = robot_bandpt_nxt->position() - robot_bandpt->position();
    Eigen::Vector2d robot_vel = diff_robot / dt_robot->dt();
    Eigen::Vector2d diff_human = human_bandpt_nxt->position() - human_bandpt->position();
    Eigen::Vector2d human_vel = diff_human / dt_human->dt();

    Eigen::Vector2d C = human_bandpt->position() - robot_bandpt->position();

    Eigen::Vector2d d_rtoh = human_bandpt->position() - robot_bandpt->position();
    Eigen::Vector2d d_htor = robot_bandpt->position() - human_bandpt->position();
    // double dir_cost = (std::max(robot_vel.dot(d_rtoh), 0.0) + std::max(human_vel.dot(d_htor), 0.0)) / d_rtoh.dot(d_rtoh);


    double ttcplus = std::numeric_limits<double>::infinity();
    double C_sq = C.dot(C);
    static double i = 0;
    static double j = 0;
    static double r_dt = 0;
    static double r_dt_miss = 0;

    C_sq = C.dot(C);

    _error[0] = 0.0;
    // std::cout << "dt_robot->dt()" << dt_robot->dt() << '\n';
    if (C_sq <= radius_sum_sq_) {
      ttcplus = 0.0;
    }
    else {
      Eigen::Vector2d V = robot_vel - human_vel;
      double C_dot_V = C.dot(V);
      // std::cout << "human_vel" << human_vel.norm() << '\n';
      if (C_dot_V > 0 && human_vel.norm() > 0) { // otherwise ttcplus is infinite
        double V_sq = V.dot(V);
        double f = (C_dot_V * C_dot_V) - (V_sq * (C_sq - radius_sum_sq_));
        if (f > 0) {         // otherwise ttcplus is infinite
           ttcplus = (C_dot_V - std::sqrt(f)) / V_sq;
         }
         else{
           ttcplus = (C_dot_V)/(C.norm()*V.norm())*((C_dot_V - std::sqrt(f)) / V_sq);
         }
       }
      }

     if (ttcplus < std::numeric_limits<double>::infinity()) {
       r_dt += dt_robot->dt();
       r_dt_miss = 0;
       i++;
       j=0;
       // std::cout << "time " << dt_robot->dt() << '\n';
     	if( r_dt >= (cfg_->hateb.ttcplus_timer) ){              // timer for number of poses to check
      	  //_error[0] = penaltyBoundFromBelow(ttcplus, cfg_->hateb.ttcplus_threshold, cfg_->optim.penalty_epsilon)/cfg_->hateb.ttcplus_threshold;
      	  _error[0] = penaltyBoundFromBelow(ttcplus, cfg_->hateb.ttcplus_threshold, cfg_->optim.penalty_epsilon);
          // std::cout << "error_[0] after penaltybound"<< _error[0] << '\n';

      	  if (cfg_->hateb.scale_human_robot_ttcplus_c) {
            _error[0] = _error[0] * cfg_->optim.human_robot_ttcplus_scale_alpha / C_sq;
      	  }
        }
     }

    else {
      // no collision possible
      j++;
      r_dt_miss += dt_robot->dt();
      if(r_dt_miss>=5*(cfg_->hateb.ttcplus_timer)){ //Check if the misses are consecutive for atleast 5 times of the timer
        i=0;
        j=0;
        r_dt=0;
        r_dt_miss=0;
      }
    	//  if(C_sq > 4){
      //   _error[0] = 0.0;
     	// }
      // _error[0] = penaltyBoundFromAbove(dir_cost, cfg_->hateb.dir_cost_threshold, cfg_->optim.penalty_epsilon);
    }

    // std::cout << "error_[0]"<< _error[0] << '\n';
     ROS_DEBUG_THROTTLE(0.5, "ttcplus value : %f", ttcplus);

     ROS_ASSERT_MSG(std::isfinite(_error[0]),
                   "EdgeHumanRobot::computeError() _error[0]=%f\n", _error[0]);
  }


  void setParameters(const HATebConfig &cfg, const double &robot_radius,
                     const double &human_radius) {
    cfg_ = &cfg;
    radius_sum_ = robot_radius + human_radius;
    radius_sum_sq_ = radius_sum_ * radius_sum_;
  }

 protected:
  double radius_sum_ = std::numeric_limits<double>::infinity();
  double radius_sum_sq_ = std::numeric_limits<double>::infinity();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

 } // end namespace

 #endif // EDGE_HUMAN_ROBOT_TTCplus_H_
