#ifndef _UNIF_FEAT_TRACKER_H_
#define _UNIF_FEAT_TRACKER_H_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <list>
#include <numeric>

#include "uniform_feat_extractor.hh"

using namespace std;
using namespace cv;

class UniformFeatureTracker
{
private:
	int _prev_frame_idx;			//
	int _curr_frame_idx;			//
	int _num_tracked_feats;			// # of features actively tracked now
	int _last_feat_id;				// ID assisgned to the most recently added feature 
									//    (this is not index, it can be any integer assigned as ID)
	int _max_num_feats;				// 
	int _max_hist_len;				// Length of the flow trails
	vector<bool> _is_initialized;	// Bookkeeps initialized _frames (images)
	
	// Optical flow trailer visualization related params.
	float _flow_thickness;	//
	bool  _use_aged_colors;	// defines whether to change older feature colors.
	cv::Scalar _flow_color;	// color of the line segment btw. corners
	cv::Scalar _feat_color;	// color of the corners
	float _feat_radius;		// radius of the circle representing a corner
	float _feat_thickness;	// negaitve corresponds to filled circle
	float _aging_time;		// frame after which aging effect starts
				
	
	vector<vector<cv::Point2f> > 
						_feat_history_grid;		// (_max_num_feats x _max_hist_len) grid in which all feats. are kept
												// This structure constitutes the very basic of the class. Its dimensions
												// are time vs feature point. Rather than reallocating memory for each new
												// feature set, we slide the pointed to another index of time.
	vector<vector<cv::Mat> > _frames;			// Images, for OF calculation (### might be unnecessary)
	vector<int> _feat_ids;						// ID of each tracked feature
	vector<int> _start_frame_idx;				// IDX of the earliest frame
												//    ex: _feat_history_grid[_start_frame_idx[i]][i] gives the position 
												//    (in pixel coordinates) of the i^th feature when it entered the list the first.
	vector<int> _feat_age;						// # of frames since the feature has been initialized
	vector<Scalar>  _feat_colors;				// Colors used in plotting flow and features. 
												// 	  Different color for different ages :)

	// Required arguments for 'calcOpticalFlowPyrLK(...)'
	vector<Point2f> _temp_feats;				// 
	vector<uchar> _status;						//
	vector<float> _errs;						//
	TermCriteria _criteria;						//
	Size2i _winSize;							// 
	Size2i _image_size;							// 
	int _max_pyra_level;						// 

	// Parameters for outlier elimination
	float _max_flow_rate;						// is the maximum OF length in one frame
	float _num_stddevs;							// is the maximum deviance of a single step OF vector can
												//    be from the mean of the complete flow trailer
	int _max_age;								// maximum life in frames after which the feature is 
												//    removed from the OF trailer
	

	int _initialize();							// initializes paramters, allocates memory etc.
	int _eliminate_outliers(const cv::Mat &mask);// 
public:
	// This constructor gets the maximum number of features to be tracked
	// and the length of the history.
	UniformFeatureTracker( int max_num_feats = 600, int max_hist_len = 2 );
	// This function tracks previously extracted features into the given 
	// new frame. It also adds new features as long as there is still room 
	// to reach to '_max_num_feats'
	int track_features(const cv::Mat &img, UniformFeatureExtractor &unifFeatExt, const cv::Mat &mask = cv::Mat(0, 0, CV_8U));
	// This functions plots the OF trailers when 'plot_flow = true' and the
	// corresponding features in the last frame if 'plot_feats = true' onto
	// the given image 'img'.
	int plot_flow(cv::Mat &img, bool plot_flow = true, bool plot_feats = true);
	// These functions return the feature points together with their IDs which
	// have age more than or equal to 'lifetime'. If 'lifetime < 0', then all
	// features are returned.
	int get_features(vector<Point2f> &features, vector<int> &feat_ids, int lifetime = -1);
	int get_features(vector<Point2d> &features, vector<int> &feat_ids, int lifetime = -1);
};

#endif
