#ifndef __LASER_PROC_HH__
#define __LASER_PROC_HH__

#include <utils.hh>
#include <calib_params.hh>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>             
#include <limits>            
#include <vector>            
#include <cassert>           
#include <iostream>          

#include <Eigen/Core>        
#include <Eigen/Dense>          
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

/*
   This class defines a set of 2D laser scanner filtering and
   processing functions such as 
   -- noise removal, {median filter}
   -- line and corner extration, {rate_linearity}
   -- general feature extraction, {line extraction}
   -- clustering, {see below}
   -- projection, {transform and projection}
   -- splitting source into multiple clusters, {masking}
   -- labeling, {clustering}
   -- covariance estimation (as in Censi's ICRA07 paper), {get_fim, uses utils function}
   -- downsampling, {see below}
   -- generate RVIZ visualizable messages, {see below}
   -- transform data from sensor to robot frame, {transform}
   -- apply general transformations, {transform}
   -- coordinate transformations. {### polar2euclidean}
   This class can be utilized inside a specific node or the wrapper
   application node can be triggered to publish the data. The data
   published in the latter case can be used by multiple nodes, hence
   prevent redundant processing. 

   The set of functions/filters affect cumulatively on the registered data.

   This class does book-keeping for the in/valid rays as a function of the 
   provided angle spans. The valid angle spans are defined inside the
   LidarParams structure. '_mask' vector encodes the validity of each ray.


 */

using namespace std;


class LaserProc{
  private:
    // This class can handle multiple scan at once. The original
    // data is saved in the '_orig_scans' vector
    double _angle_min, _angle_max, 
            _angle_increment;           // same as the properties of the laser scan ROS message fields
    vector<double> _ranges;             // stores the ranges. 
    vector<double> _temp_ranges;        // used in median filter and other function where the original data has to be kept
                                        // untouched till the end of the process.
    LidarCalibParams _params;           // saves a copy of the parameter set.
    vector<Eigen::Vector3d> _3d_rays;   // saves the transformed 3D point set.
    vector<Eigen::Vector2d> _2d_rays;   // saves the projected 2D point set.
    vector<int> _mask;                  // encodes validity of range data saved in '_data'.
    // Also saves the cluster id's.
    vector<double> _linearity;          // encodes whether a point belongs to a line of corner.
    vector<pair<int, int> > _line_idxs; // saves the indices of the end points of lines.
    Eigen::Matrix3d _fim;               // Fisher Information Matrix.

    // This stores a set of distinctive colors for RVIZ visualization.
    vector<Eigen::Vector3i> _colors;
    
    inline void _generate_colors(){
      int temp_colors[] = {
      0x000000, 0x00FF00, 0x0000FF, 0xFF0000,
      0x01FFFE, 0xFFA6FE, 0xFFDB66, 0x006401,
      0x010067, 0x95003A, 0x007DB5, 0xFF00F6,
      0xFFEEE8, 0x774D00, 0x90FB92, 0x0076FF,
      0xD5FF00, 0xFF937E, 0x6A826C, 0xFF029D,
      0xFE8900, 0x7A4782, 0x7E2DD2, 0x85A900,
      0xFF0056, 0xA42400, 0x00AE7E, 0x683D3B,
      0xBDC6FF, 0x263400, 0xBDD393, 0x00B917,
      0x9E008E, 0x001544, 0xC28C9F, 0xFF74A3,
      0x01D0FF, 0x004754, 0xE56FFE, 0x788231,
      0x0E4CA1, 0x91D0CB, 0xBE9970, 0x968AE8,
      0xBB8800, 0x43002C, 0xDEFF74, 0x00FFC6,
      0xFFE502, 0x620E00, 0x008F9C, 0x98FF52,
      0x7544B1, 0xB500FF, 0x00FF78, 0xFF6E41,
      0x005F39, 0x6B6882, 0x5FAD4E, 0xA75740,
      0xA5FFD2, 0xFFB167, 0x009BFF, 0xE85EBE};
      _colors.clear();
      _colors.reserve(64);
      for(int i = 0 ; i < 64 ; i++){
        _colors[i](0) = (temp_colors[i] & 0xFF0000) >> 16;
        _colors[i](1) = (temp_colors[i] & 0x00FF00) >> 8;
        _colors[i](2) = (temp_colors[i] & 0x0000FF);
      }
    }

    void _initialize();
  public:
    LaserProc();
    // This function updates the internal raw copy of the lidar scan and 
    // sets the internal parameters such as min/max range, dead regions etc.
    // '_mask' is initialized according to the size and the given parameter set
    // to exclude short/long/inf/NaN measurements and dead regions.
    // Returns a constant reference to '_mask'.
    const vector<int>& update_data(const sensor_msgs::LaserScan &data, const LidarCalibParams &params);
    // This function is the same as the above function except that this does 
    // not update the paramters. This function can be used if the
    // same device as the previous frame is used.
    // Returns a constant reference to '_mask'.
    const vector<int>& update_data(const sensor_msgs::LaserScan &data);
    // This function applies median filter on the range data.
    // 'win_size' determines the size of the filter window.
    // Invalid ranges are not included in the process.
    // Returns a constant reference to the ranges vector.
    const vector<double>& median_filter(int win_size = 3);
    // This function removes rays such that any two ray tips are no closer
    // than 'range_thres' meters to each other.
    // Returns a constant reference to '_mask'.
    const vector<int>& downsample(double range_thres = 0.05);
    // This function detects and removes rays reflected from corners.
    // These kind of rays typically occur when the scanner ray scuffs
    // an egde and causes a range in the middle of the adjacent ranges.
    // The adjacent ranges are usually significantly different from eachother.
    // Returns a constant reference to '_mask'. 'range_thres' is the min.
    // difference in range values of consecutive rays. The ray with the 
    // greater range is marked as invalid. It is recommended to run
    // shadow filter before any other filter.
    const vector<int>& remove_shadows(double range_thres = 0.50);
    // This function clusters the laser data. Each ray label is stored in 'mask'. 
    // Clustering is done accoring to a set of parameters:        
    // -- _mask         : holds the cluster id's for all the data. '_mask[] = 0' means the ray is invalid.
    // -- _num_clusters : the number of clusters.
    // -- min_skips    : consecutive this many invalid rays cause initialization of a new cluster
    // -- range_jump   : a change in range value between consecutive rays causes init. of a new cluster
    // -- min_cluster_size : rays belonging to a cluster with number of elements less than this are marked as unused (=0)
    // Rays with 'mask[] == 0' are never assigned to another cluster.
    // Returns a constant reference to '_mask'.
    const vector<int>& clusters(int min_skips, double range_jump, int min_cluster_size);
    // This function rates all of the points between 0-to-1 where '0' means a corner
    // and '1' a line. 'win_size' is the number of rays on each side of ray of concern
    // in 'line-ness' rating. Invalid points are not included in the calculation.
    // Thus, this function behaves different depending on the order of applied filters.
    // Returns a constant reference to '_linearity'.
    // The return array is of the same as the '_mask' array.
    const vector<double>& rate_linearity(int win_size = 3);
    // This function uses the built-in OpenCV 'approxPolyDP' function to fit lines.
    // Previous calls to filtering functions have direct effect on the output of this
    // function. This is because only valid points are included in the process.
    // It is guaranteed that no line passes from an invalid point. 
    // The resultant set of lines with lenght '< min_len' and with number of points
    // '< min_pts' are removed from the line list. 'epsilon' is the naximum distance 
    // between the original curve and its approximation (defn. from OpenCV doc.). 
    // Previous call to 'rate_linerity()' does not have any effect on the output.
    // Returns a constant reference to the vector of end point indices of the lines.
    // Other getter function have to used in order to find out 2/3D/range correspondences
    // of each end point.
    const vector<pair<int, int> >& extract_lines(double min_len = 0.10, double min_pts = 3, double epsilon = 0.08);
    // This function transforms points from sensor coordinate frame to the 
    // new frame given by the SE(3) object 'trans'. If 'is_sensor_pose = true'
    // then the 'trans' defines a mapping from the sensor frame to the new (world)
    // frame. Otherwise, 'trans' is interpreted as the pose of the robot. In the
    // latter case relative pose of the sensor w.r.t. the robot is integrated 
    // too. The relative pose is given through the parameters structure.
    // Any filter applied after a transformation will not update the set of
    // 3D points. In order to reflect the effect, a second call to this function
    // is required. This also returns a reference to the transformed 3D points.
    // The return array is of the same as the '_mask' array.
    const vector<Eigen::Vector3d>& transform(const Eigen::Matrix4d &trans, bool is_sensor_pose = false);
    // This function projects points onto a plane with the normal vector 'n'. 
    // Previously applied transformations have affect on the projected point
    // set. This also returns are reference to the projected 2D points.
    // The returned array is of the same size as the '_mask' array.
    const vector<Eigen::Vector2d>& project(const Eigen::Vector3d &n = Eigen::Vector3d(0, 0, 1));
    // This function returns a constant referecen to the ranges array.
    // The vector is of the same size a the '_mask' array.
    const vector<double>& get_ranges();
    // This function returns the Fisher Information Matrix calculated 
    // using the method given in Andrea Censi's ICRA07 paper. This takes
    // into account of the latest clusters/mask vectors.
    const Eigen::Matrix3d& get_fim();
    // This returns a constant reference to the internal '_mask' array.
    const vector<int>& get_mask_array();
    // This returns the transformed (if any) set of 3D points. The array 
    // is of the same size as the '_mask' array.
    const vector<Eigen::Vector3d>& get_3d_points();
    // This returns the result of 'rate_linearity()' function. The return
    // vector is of the same size as the '_mask' array.
    const vector<double>& get_linearity_rates();
    // This returns the results of 'project()' function. The return
    // vector is of the same size as the '_mask' array.
    const vector<Eigen::Vector2d>& get_projections();
    // This function returns a MarkerArray. Each Marker is populated with 
    // the points from one cluster only. The 'i^th' marker has the points
    // from the 'i^th' cluster. The last four markers have all the
    // points, linearity rates, line segments and the covariance estimate
    // respectively.
    bool get_RVIZ_markers(visualization_msgs::MarkerArray &marray);
};



#endif
