/*
   This header file defines a set of mathematics utility 
   functions. Additionally, other header files which define
   further utility functions such as representation transformations,
   lidar data processing. The user should include this header file
   to benefit from all the utiity function while keeping in mind
   the namespace structure.
 */

#ifndef PI
#define PI 3.14159265359
#endif

#include <cmath>
#include <limits>
#include <vector>
#include <cassert>
#include <iostream>

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>

#include "trans_utils.hh"
#include "laser_utils.hh"

using namespace std;
using namespace Eigen;

// Putting this header lock at the beginning of the file prevents
// including the library headers in the preamble of this file.
// However functions in the "trans_utils" and "laser_utils" headers
// utilizes these libraries.

#ifndef _UTILS_HH_
#define _UTILS_HH_

// Define new Eigen matrix and vector types
namespace Eigen{
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 5, 1> Vector5d;
}

namespace utils{
  // This macro provides an efficient way to raise exceptions with
  // very informative command line messages.
#define ASSERT(condition, message) \
  if (! (condition)) { \
    std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
    << " line " << __LINE__ << ": " << message << std::endl; \
    std::exit(EXIT_FAILURE); \
  } \

#ifndef DEG2RAD
  #define DEG2RAD(x) ((x) / 180.0 * PI)
#endif
#ifndef RAD2DEG
  #define RAD2DEG(x) ((x) / PI * 180.0)
#endif

  // The 'utils' namespace implements utility functions which cannot
  // be categorized into a specific class of helper routines. However
  // functions such as rotation transformations and ROS-Eigen data
  // structure conversions are collected under the 'utils::trans'
  // namespace in its specific header and source files. Similarly
  // laser processing routines are grouped under 'utils::laser'
  // namespace. As new functions are required, they should be added
  // directly under 'utils' namespace unless they intuitively form
  // a group/set of functions.


  // Clamps the given value in between the extrema
  inline double clamp(double val, double min, double max){
    return val > max ? max : val < min ? min : val;
  }

  /*
     inline void clamp(double &val, double min, double max){
     val = val > max ? max : val < min ? min : val;
     }
   */

  // This function returns the 2*PI modula shifted by -PI
  // of an angle in radians
  inline double fix_angle(double ang){
    while(ang > PI)
      ang -= PI;
    while(ang <= -PI)
      ang += PI;
    return ang;
  }

  inline void fix_angle(double *ang){
    while(*ang > PI)
      *ang -= PI;
    while(*ang <= -PI)
      *ang += PI;
  }
}

#endif

