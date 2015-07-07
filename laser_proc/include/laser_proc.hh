#ifndef __LASER_PROC_HH__
#define __LASER_PROC_HH__

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

#include <utils.hh>
#include <calib_params.hh>

#include <opencv2/opencv.hpp>

/*
   This class defines a set of 2D laser scanner filtering and
   processing functions such as 
   -- noise removal, {median filter}
   -- line and corner extration, {rate_linearity}
   -- general feature extraction, {line extraction}
   -- clustering, {see below, cluster according to spatial properties}
   -- projection, {transform and projection}
   -- splitting source into multiple clusters, {see masking}
   -- labeling, {see clustering, masking}
   -- covariance estimation (as in Censi's ICRA07 paper)
   -- downsampling, {see below}
   -- generate RVIZ visualizable messages, {see below}
   -- transform data from sensor to robot/world frame, {transform}
   -- apply general transformations, {see transform}
   -- coordinate transformations. {polar2euclidean}
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
    double _angle_min, _angle_max, 
           _angle_increment,
           _sec, _nsec;                // same as the ROS LaserScan message fields.
    vector<double> _intensities;		// '' 
    vector<double> _ranges;             // stores the ranges. Overwritten if necessary.
    vector<double> _thetas;				// stores the array of angles corresponding to each range value.
    vector<double> _temp_ranges;        // used in median filter and other function where the original 
    // data has to be kept unmodified till the end of the process, filtering.
    LidarCalibParams _params;           // saves a copy of the parameter set.
    vector<Eigen::Vector3d> _3d_rays;   // saves the transformed 3D point set.
    vector<Eigen::Vector2d> _2d_rays;   // saves the 2D Euclidean coordinates of the ray tips in sensor frame.
    vector<int> _mask;                  // encodes validity of range data saved in '_ranges'.
    // Also saves the cluster id's.
    vector<double> _linearity;          // encodes whether a point belongs to a line or a corner.
    // '0' means line and '1' corner.
    vector<pair<int, int> > _line_idxs; // saves the indices of the end points of lines.
    Eigen::Matrix3d _fim;               // Fisher Information Matrix.
    int _num_clusters;                  // # of clusters starting from 0.

    vector<double> _alphas;             // angles used to calculate fim in 'get_fim(...)'.


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

    // These save whether '_3d_rays' and '_2d_rays' vectors needs
    // to be recalculated.
    bool _2d_euclidean_coords_valid;
    bool _3d_euclidean_coords_valid;

    // This function initializes the internal vectors and parameters.
    // Whenever new data arrives, this function is called.
    void _initialize();
    // These functions transformes the range data to their 2D/3D Euclidean
    // representations.
    void _polar_to_2d_euclidean();
    void _polar_to_3d_euclidean();

  public:
    LaserProc();
    // This function updates the internal raw copy of the lidar scan and 
    // sets the internal parameters such as min/max range, dead regions etc.
    // '_mask' is initialized according to the data size and the given parameter set
    // such that it excludes short/long/inf/NaN measurements and dead regions.
    // Also rays redirected upwards and downwards are labeled as '-1' and '-2'
    // in the '_mask' array. These clusters are treated specially in many of the
    // other porcesses and filters. 
    // Returns a constant reference to '_mask'.
    const vector<int>& update_data(const sensor_msgs::LaserScan &data, const LidarCalibParams &params);
    // This function is the same as the above function except this does 
    // not update the paramters. This function can be used if the
    // same device as the previous frame is used.
    // Returns a constant reference to '_mask'.
    const vector<int>& update_data(const sensor_msgs::LaserScan &data);
    // This function applies median filter on the range data.
    // 'win_size' determines the size of the filter window.
    // Invalid ranges are not included in the process.
    // Returns a constant reference to the ranges vector.
    const vector<double>& median_filter(int win_size = 5);
    // This function masks out laser beams with intensity lower
    // than 'intensity_thres'. This filter can be useful when 
    // working in environment which have low-reflectance surfaces
    // and fail the tailed estimators.
    const vector<int>& intensity_filter(double intensity_thres = 1000);
    // This function masks out the points those are greater (smaller) than
    // 'linearity_thres' if 'upper_bound == true' ('upper_bound == false').
    // This returns a constant reference to the resultant '_mask' vector.
    // The programmer is assumed to have called  'rate_linearity()' before
    // this function is called.
    const vector<int>& linearity_filter(double linearity_thres = 0.3, bool upper_bound = true);
    // This function removes rays such that no two ray tips are closer
    // than 'range_thres' meters to each other.
    // Returns a constant reference to '_mask'.
    const vector<int>& downsample(double range_thres = 0.05);
    // This function checks for significant jumpes on the ranges.
    // If consecutive rays differ in their ranges significantly, 
    // the farther point is assumed to be occluded 
    // (See LOAM paper Zhang, Singh). If 'win_size > 1' then 
    // 'win_size - 1' number of neighbors of the center point are 
    // marked as invalid as well. 'range_thres' is the min. difference
    // between range values of consecutive rays. It is recommended to run
    // occusion filter before any other filters. This function 
    // returns a constant reference to '_mask'. 
    const vector<int>& remove_occlusions(double range_thres = 0.50, int win_size = 3);
    // This function removes points on edges hit with large
    // angle of attack where angle of attack is the relative orinetation
    // between the normal of the corresponding edge and the ray in consideration.
    // All rays those have greater attack angles than 'angle_thres' are
    // removed together with the 'win_size' number of neighbors from
    // each side. This function returns a constant reference to the
    // '_mask' vector. (1.4835 rad. ~ 85 deg.)
    const vector<int>& remove_slant_edges(double angle_thres = 1.4835, int win_size = 3);
    // This function clusters the laser data. Each ray label is stored in 'mask'. 
    // Clustering is carried accoring to a set of parameters:        
    // -- _mask         : holds the cluster id's for all the data. '_mask[] = 0' means the ray is invalid.
    // -- _num_clusters : the number of clusters.
    // -- min_skips    : consecutive this many invalid rays cause initialization of a new cluster
    // -- range_jump   : a change in range value between consecutive rays causes init. of a new cluster
    // -- min_cluster_size : rays belonging to a cluster with number of elements less than this are marked as unused (=0)
    // Rays with 'mask[] == 0' are never assigned to another cluster.
    // Returns a constant reference to '_mask'.
    const vector<int>& cluster(int min_skips = 3, double range_jump = 0.50, int min_cluster_size = 3);
    // This function rates all of the points between [0-inf) where greater values mean a corner
    // and '0' a line. 'win_size' is the number of rays on each side of ray of concern
    // in linearity rating. Invalid points are not included in the calculation.
    // Thus, this function behaves different depending on the order of applied filters.
    // Returns a constant reference to '_linearity'.
    // The return array is of the same as the '_mask' array.
    const vector<double>& rate_linearity(int win_size = 5);
    // This function uses the built-in OpenCV 'approxPolyDP' function to fit lines.
    // Previous calls to filtering functions have direct effect on the output of this
    // function. This is because only valid points are included in the process.
    // It is guaranteed that no line passes from an invalid point. 
    // The resultant set of lines with lenght '< min_len' and with number of points
    // '< min_pts' are removed from the line list. 'epsilon' is the naximum distance 
    // between the original curve and its approximation (defn. from OpenCV doc.). 
    // Previous call to 'rate_linerity()' does not have any effect on the output.
    // Returns a constant reference to the vector of the end point indices of the lines.
    // Other getter functions have to be used in order to find out the 2/3D/range correspondences
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
    const vector<Eigen::Vector3d>& transform(const Eigen::Matrix4d &trans, bool is_sensor_pose);
    // This function projects points onto a plane with the normal vector 'n'. 
    // Previously applied transformations do have affect on the projected point
    // set. This also returns are reference to the projected 2D points.
    // The returned array is of the same size as the '_mask' array.
    // 'project(...)' function is treated as a transformation and its results
    // are stored in '_3d_rays' vector. Hence projections can be retreived
    // using the 'get_3d_points(...)' function. 
    const vector<Eigen::Vector3d>& project(const Eigen::Vector3d &n = Eigen::Vector3d(0, 0, 1));
    // This function returns a constant reference to the ranges array.
    // The vector is of the same size a the '_mask' array.
    const vector<double>& get_ranges() const;
    // This function returns the angle coordinates of the 
    // ranges. This is mostly used with other functions suchs as
    // utils::laser::register(...). As long as the laser source
    // does not chance, angles are not re-calculated hence does not
    // cause extra load.
    const vector<double>& get_thetas() const;
    // This function returns the Fisher Information Matrix calculated 
    // using the method given in Andrea Censi's ICRA07 paper. This takes
    // into account of the latest clusters/mask vectors.
    const Eigen::Matrix3d& estimate_fim(double skip_dist = 0.30, int skip_idxs = 30);
    // This returns FIM estimated in 'estimate_fim(...)'
    const Eigen::Matrix3d& get_fim() const;
    // This returns a constant reference to the internal '_mask' array.
    const vector<int>& get_mask() const;
    // This returns the transformed (if any) set of 3D points. The array 
    // is of the same size as the '_mask' array.
    const vector<Eigen::Vector3d>& get_3d_points() const;
    // This returns the result of 'rate_linearity()' function. The returned
    // vector is of the same size as the '_mask' array.
    const vector<double>& get_linearity_rates() const;
    // This function returns a MarkerArray. Each Marker is populated with 
    // the points from one cluster only. The 'i^th' marker has the points
    // from the 'i^th' cluster. The last four markers have all the
    // points, linearity rates, line segments and the covariance estimate
    // respectively. The set of points reflect the effect of previous 
    // transformations.
    bool get_RVIZ_markers(visualization_msgs::MarkerArray &marray) const;
    // This returns the extrated line indices.
    const vector<pair<int, int> >& get_line_indexes() const;
    // This returns the calibration parameters structure
    const LidarCalibParams& get_calib_params() const {
      return _params;
    }
    // This returns the timestamp of the original laser scan.
    ros::Time get_timestamp() const {
      return ros::Time(_sec, _nsec);
    }
    // This returns the number of clusters
    int get_num_clusters() const{
      return _num_clusters;
    }
    double get_angle_min() const { return _angle_min;}
    double get_angle_max() const { return _angle_max;}
    double get_angle_increment() const { return _angle_increment;}
};



#endif
