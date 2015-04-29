#ifndef __LASER_UTILS_HH__
#define __LASER_UTILS_HH__

#include "utils.hh"

namespace utils{
	namespace laser{
		// The following function transforms the point representation
		// from polar coordinates to Euclidean coordinates. 'r' is the
		// range and 'th' is the direction of the point. The Euclidean
		// coordinates are returned through 'x' and 'y'.
		inline bool polar2euclidean(const double &r, const double &th, double &x, double &y){
			x = r * cos(th);
			y = r * sin(th);
			return true;
		}

		// The following function transforms the point representation
		// from polar coordinates to Euclidean coordinates in batch.
		// 'rs' is the vector of ranges, 'ths' is the vector of corresponding
		// orientations. Only the points where 'mask[i] == true' are converted.
		// If 'mask.size() == 0' then all the points are processed. If 'mask.size()
		// != rs.size()' or 'rs.size() != ths.size()' a runtime error is fired.
		inline bool polar2euclidean(const vector<double> &rs, const vector<double> &ths, const vector<char> &mask, vector<double> &xs, vector<double> &ys){
			ASSERT(rs.size() == ths.size() &&
					(mask.size() == 0 || mask.size() == rs.size()), 
					"ranges.size == thetas.size() && (mask.size() == 0 || mask.size() == ranges.size())");
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

		// This function estimates the Fisher Information Matrix for a given
		// laser scanner data using Andre Censi's method (ICRA-07). When constructing
		// FIM, only data points with 'mask[i] == cluster_id' are used. If 'mask.size() == 0'
		// all the points are included in FI. If 'mask.size() != data.size()' a runtime error is fired.
		bool get_fim(const sensor_msgs::LaserScan   &data, const vector<char> &mask, Eigen::Matrix3d &fim, const char &cluster_id = 1);
		// ### This function estimates the Fisher Information Matrix for a given
		// 3D point cloud using a derivative of Andrea Censi's method (ICRA-07)
		// When constructing FIM, only data points with 'mask[i] == cluster_id' are used. 
		// If 'mask.size() == 0' all the points are included in FI. If 'mask.size() !=
		// data.size()' a runtime error is fired.
		bool get_fim(const sensor_msgs::PointCloud2 &data, const vector<char> &mask, Eigen::Matrix6d &fim, const char &cluster_id = 1);

		// This function clusters the laser data. Each ray label is stored in 'mask'. 
		// Clustering is done accoring to a set of parameters: 
		// -- (O) mask		   : an output array of cluster values 
		// -- (O) num_clusters : the number of clusters as an output value
		// -- (I) min_range    : any range smaller than this is marked as unused (=0)
		// -- (I) max_range    : any range larger than this is marked as unused (=0)
		// -- (I) min_skips    : consecutive this many rays cause initialization of a new cluster
		// -- (I) range_jump   : a change in range value between consecutive rays causes init. of a new cluster
		// -- (I) min_cluster_size : rays belonging to a cluster with number of elements less than this are marked as unused (=0)
		// For some cases, certain angle spans should be excluded from clustering (in turn, estimation)
		// due to occlusion etc. If 'num_clusters = 1' and 'mask' has the same size of the rays vector,
		// rays with 'mask[] == 0' are excluded from the clustering process.
		bool cluster_laser_scan(const sensor_msgs::LaserScan &data, vector<char> &mask, int &num_clusters,
				double min_range = 0.10, double max_range = 20.0, int min_skips = 5,
				double range_jump = 0.3, int min_cluster_size = 3);

		// This function uses the built-in OpenCV 'approxPolyDP' function to fit lines
		// to a given laser data. If 'mask.size() == 0' all of the data is processed.
		// If 'mask.size() == data.size()' then only the points with 'mask[i] != false'
		// are processed. Otherwise a runtime error is fired. 'num_lines' return the 
		// total number of lines extracted. 'mask' is populated with the line numbers
		// each point belongs to. Lines with length '< min_len' and fit from number of
		// points '< min_pts' are masked with zero. This function works best after 
		// preprocessing with 'cluster_laser_scan(...)'. However this function overwrites
		// the 'mask' vector and the user should save it. 'epsilon' is the naximum distance
		// between the original curve and its approximation (defn. from OpenCV doc.).
		bool fit_lines(const sensor_msgs::LaserScan &data, const vector<char> &mask, vector<pair<int, int> > &line_idxs,
						double min_len = 0.10, double min_pts = 3, double epsilon = 0.08);

	}
}

#endif
