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
 *  Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
 *********************************************************************/


#ifndef HUMAN_PATHARRAY_DISPLAY_H
#define HUMAN_PATHARRAY_DISPLAY_H

// #include <nav_msgs/Path.h>
#include <human_msgs/HumanPathArray.h>
#include <human_msgs/HumanPath.h>

#include "rviz/message_filter_display.h"
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>

namespace Ogre
{
class ManualObject;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class BillboardLine;
class VectorProperty;
}

namespace rviz_path_array
{

class HumanPathArrayDisplay;
/**
 * \class HumanPathArrayDisplay
 * \brief Displays a human_msgs::HumanPathArray message
 */
class HumanPathArrayDisplay : public rviz::MessageFilterDisplay<human_msgs::HumanPathArray>
{
Q_OBJECT
public:
  HumanPathArrayDisplay();
  ~HumanPathArrayDisplay() override;

  /** @brief Overridden from Display. */
  void reset() override;

protected:
  /** @brief Overridden from Display. */
  void onInitialize() override;

  /** @brief Overridden from MessageFilterDisplay. */
  void processMessage(const human_msgs::HumanPathArray::ConstPtr& msg) override;

private Q_SLOTS:
  void updateBufferLength();
  void updateStyle();
  void updateLineWidth();
  void updateOffset();
  void updatePoseStyle();
  void updatePoseAxisGeometry();
  void updatePoseArrowColor();
  void updatePoseArrowGeometry();

private:
  void destroyObjects();
  void allocateArrowVector(std::vector<rviz::Arrow*>& arrow_vect, int num);
  void allocateAxesVector(std::vector<rviz::Axes*>& axes_vect, int num);
  void destroyPoseAxesChain();
  void destroyPoseArrowChain();

  std::vector<Ogre::ManualObject*> manual_objects_;
  std::vector<rviz::BillboardLine*> billboard_lines_;
  std::vector<std::vector<rviz::Axes*> > axes_chain_;
  std::vector<std::vector<rviz::Arrow*> > arrow_chain_;

  rviz::EnumProperty* style_property_;
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* line_width_property_;
  rviz::IntProperty* buffer_length_property_;
  rviz::VectorProperty* offset_property_;

  enum LineStyle
  {
    LINES,
    BILLBOARDS
  };

  // pose marker property
  rviz::EnumProperty* pose_style_property_;
  rviz::FloatProperty* pose_axes_length_property_;
  rviz::FloatProperty* pose_axes_radius_property_;
  rviz::ColorProperty* pose_arrow_color_property_;
  rviz::FloatProperty* pose_arrow_shaft_length_property_;
  rviz::FloatProperty* pose_arrow_head_length_property_;
  rviz::FloatProperty* pose_arrow_shaft_diameter_property_;
  rviz::FloatProperty* pose_arrow_head_diameter_property_;

  enum PoseStyle
  {
    NONE,
    AXES,
    ARROWS,
  };
};

} // namespace rviz

#endif /* HUMAN_PATHARRAY_DISPLAY_H */
