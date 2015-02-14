#ifndef _UTILS_HH_
#define _UTILS_HH_

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

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace Eigen;

namespace Eigen{
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}

namespace utils{

Matrix3d yaw2dcm(const double &yaw);
Matrix3d quat2dcm(const Vector4d &quat);
Vector3d quat2rpy(const Vector4d &quat);
Matrix3d rpy2dcm (const Vector3d &rpy);
Vector4d rpy2quat(const Vector3d &rpy);
Vector4d dcm2quat(const Matrix3d &dcm);
Vector3d dcm2rpy (const Matrix3d &dcm);

Matrix3d cancel_yaw(const Matrix3d &dcm );
Vector4d cancel_yaw(const Vector4d &quat);

// ###
Vector3d slerp(const Vector3d &quat1, const Vector3d &quat2, double theta);


inline double saturate(double val, double min, double max){
	return val > max ? max : val < min ? min : val;
}

inline double fix_angle(double ang){
	while(ang > PI)
		ang -= PI;
	while(ang <= -PI)
		ang += PI;
	return ang;
}

inline bool polar2euclidean(const double &r, const double &th, double &x, double &y){
	x = r * cos(th);
	y = r * sin(th);
	return true;
}

inline bool polar2euclidean(const vector<double> &rs, const vector<double> &ths, const vector<char> &mask, vector<double> &xs, vector<double> &ys){
	assert(rs.size() == ths.size() &&
		   (mask.size() == 0 || mask.size() == rs.size()));
	if(rs.size() != xs.size())
		xs.resize(rs.size());
	if(rs.size() != ys.size())
		ys.resize(xs.size());
	bool use_mask = mask.size() != 0;
	for(int i = 0 ; i < (int)rs.size() ; i++)
		if(use_mask && mask[i] == true)
			polar2euclidean(rs[i], ths[i], xs[i], ys[i]);
	return true;
}

bool get_fim(const sensor_msgs::LaserScan   &data, const vector<char> &mask, Eigen::Matrix3d &fim, const char &cluster_id = 1);
bool get_fim(const sensor_msgs::PointCloud2 &data, const vector<char> &mask, Eigen::Matrix6d &fim, const char &cluster_id = 1);

// This function clusters the laser data. Each ray label is stored in 'mask'. 
// Clustering is done accoring to a set of parameters: 
// -- mask		   : an output array of cluster values 
// -- num_clusters : the number of clusters as an output value
// -- min_range    : any range smaller than this is marked as unused (=0)
// -- max_range    : any range larger than this is marked as unused (=0)
// -- min_skips    : consecutive this many rays causes initialization of a new cluster
// -- range_jump   : a change in range value between consecutive rays causes init. of a new cluster
// -- min_cluster_size : rays belonging to a cluster with number of elements less than this are marked as unused (=0)
// For some cases, certain angles spans are required to be excluded from clustering (actually estimation)
// due to occlusion etc. If the 'num_clusters = 1' and 'mask' has the same size of the number of rays,
// rays with 'mask[] == 0' are excluded from the clustering process.
bool cluster_laser_scan(const sensor_msgs::LaserScan &data, vector<char> &mask, int &num_clusters,
						double min_range = 0.10, double max_range = 20.0, int min_skips = 5,
						double range_jump = 0.3, int min_cluster_size = 3);


}

#endif

