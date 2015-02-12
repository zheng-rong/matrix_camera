/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010 Jack O'Quin
 *  Copyright (c) 2012 Markus Achtelik
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
 *   * Neither the name of the author nor other contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
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
 *********************************************************************/

#ifndef _FEATURES_H_
#define _FEATURES_H_

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

#include "mv_camera/MVCameraConfig.h"
typedef mv_camera::MVCameraConfig Config;

/** @file

 @brief MVCamera features interface

 @author Markus Achtelik
 */

/** @brief MVCamera Features class

 Handles the features of Matrix Vision BlueXXX cameras.

 */

struct ImageInfo
{
  double exposure_time;
  double gain;
  int width;
  int height;
  std::string color_coding;
};

inline double usToS(const int & us)
{
  return static_cast<double>(us) * 1.0e-6;
}

inline int sToUs(const double & s)
{
  return static_cast<int>(s * 1.0e6);
}

inline double kHzToMHz(const int & kHz)
{
  return static_cast<double>(kHz) * 1.0e-3;
}

inline int MHzToKHz(const double & MHz)
{
  return static_cast<int>(MHz * 1.0e3);
}

template<class T>
  T clamp(const T& value, const T& min, const T& max)
  {
    if (value > max)
      return max;
    else if (value < min)
      return min;
    else
      return value;
  }

template<class TFeature, class TVal>
  bool configure(const TFeature & feature, const TVal & value_suggested, TVal * value_returned = NULL)
  {
    //mvIMPACT::acquire::Property feature;
    if (!(feature.isWriteable() && feature.isVisible()))
    {
      if (value_returned)
        *value_returned = feature.read();
      return false;
    }
    else
    {
      if (feature.hasMaxValue() && feature.hasMinValue())
      {
        TVal val = clamp(value_suggested, feature.read(mvIMPACT::acquire::plMinValue),
                         feature.read(mvIMPACT::acquire::plMaxValue));
        feature.write(val);
        //                      ROS_INFO_STREAM("min/max defined for "<<feature.name());
      }
      else
      {
        feature.write(value_suggested);
        //                      ROS_INFO_STREAM("min/max NOT defined for "<<feature.name());
      }
      if (value_returned)
        *value_returned = feature.read();

      return true;
    }

  }


class Features
{
public:

  Features(mvIMPACT::acquire::Device *cam);

  bool initialize(Config *newconfig);
  void reconfigure(Config *newconfig);
  double getFPS()
  {
    return fps_;
  }

  ImageInfo & getImageInfo()
  {
    return image_info_;
  }
   // TODO: ugly .. find other way to gather that info

private:
  typedef int state_t; ///< mv_camera::MVCamera_* state values

  bool setFramerate(const double & fps_suggested, double * fps_returned = NULL);
  bool setHDR(const std::string & hdr_suggested, std::string * hdr_returned = NULL);
  bool setColorCoding(const std::string & cc_suggested, std::string * cc_returned = NULL);
  bool setPixelClock(const double & px_clock_suggested, double * px_clock_returned = NULL);
  double computeFrameTime();

  mvIMPACT::acquire::Device *cam_; ///< current camera
  double fps_;
  ImageInfo image_info_;
  Config oldconfig_; ///< previous Config settings
  static const double TRIGGER_PULSE_WIDTH = 2.0e-4;
};

#endif // _FEATURES_H_
