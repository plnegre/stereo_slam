/**
 * @file
 * @brief Constants
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <ros/package.h>

using namespace std;

namespace slam
{

  static const string WORKING_DIRECTORY = ros::package::getPath("stereo_slam") + "/output/";

  static const int LC_MIN_INLIERS = 20;

  static const int LC_MAX_INLIERS = 100;

  static const int LC_NEIGHBORS = 4;

  static const int LC_DISCARD_WINDOW = 10;

  static const float LC_MAX_EDGE_DIFF = 2.0;

  static const float TRACKING_MIN_OVERLAP = 80;

  static const int MIN_CLOUD_SIZE = 100;

  static const float LC_EPIPOLAR_THRESH = 4.0;

  static const float STEREO_EPIPOLAR_THRESH = 1.5;

  /*
  DEFAULT VALUES ARE:
  LC_MIN_INLIERS        = 20
  LC_MAX_INLIERS        = 100
  LC_NEIGHBORS          = 4
  LC_DISCARD_WINDOW     = 10
  TRACKING_MIN_OVERLAP  = 85
  */

} // namespace

#endif // CONSTANTS_H