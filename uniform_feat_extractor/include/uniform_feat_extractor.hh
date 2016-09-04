#ifndef _UNIF_FEAT_EXTRACTOR_H_
#define _UNIF_FEAT_EXTRACTOR_H_

#include <opencv2/opencv.hpp>
#include <utils.hh>

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

using namespace std;
using namespace cv;

class UniformFeatureExtractor
{
private:
	// Visualization related parameters
  cv::Scalar _feat_color;		// color of the feature
  cv::Scalar _grid_color;		// color of the grid borders
	float  _radius;			// size of the circle corr. to features
	float  _feat_thickness;	// thickness of the circle, negative draws filled circle
	float  _grid_thickness;	// thickness of the grid guides.
	int    _type;			// pixel connectivity

	// Size of the grid (if used)
	Size2i _grid_size;  // number of patches in width, height directions
	Size2f _patch_size; // size of each cell in pixels.

	// The generic OpenCV FeatureDetector lets initalizing detectors
	// of different times in run time. 
	string _detector_type;	// any of FAST, GFTT or HARRIS.
									        // as the type of the detector. Example "GridFast"
	FastFeatureDetector			// Two different detectors have to be defined
			_fast_detector;			// in order to handle possible user choices.
	GoodFeaturesToTrackDetector 
			_gftt_detector;
	cv::Mat _mask;	        // used to mask regions which are closer to at least one of
									        // the corners in the initial features list.
	int    _max_num_feats;	// limit output of extract_features(...) 
	double _threshold;			// feature detector threshold
	float  _min_dist;				// the minimum mutual distance of each feature in the initial 
									        // feature list in order a new feature to be added to the
									        // output feature list
	cv::Mat _histogram;			// keeps the total number of features in each grid.

	bool _force_uniformity;			// can be set to 'false' for performance concerns.

	vector<KeyPoint> _keypoints;  // container user in 'extract_features(...)'
	// This function counts the total number features in each grid into '_histogram'.
	// '_histogram' then is used in satisfying the feature distribution uniformity.
	bool _profile_grids(const vector<Point2f> &features);
public:
	// - min_dist := No feats. are at a closer distance to eachother smaller than this
	// This prevents unnecessary memory and CPU usage as well as contradicting information for higher 
	// level processes
	// - max_num_feats := Extractor tries to distribute this number optimally between image regions (patches)
	// Assigning a high value reduces performance, whereas a small value affect the estimator performance eventually.
	// - grid_size := Number of patches (regions) to which 'max_num_feats' is distributed to. This defines
	// a grid.
	// - threshold := Feature extractor threshold. By default FAST is used while the alternative is 
	// Shi-Tomasi, Harris etc.
	UniformFeatureExtractor(string detector = string("FAST"), double min_dist = 3,
							int max_num_feats = 250, Size2i grid_size  = Size2i(4, 3), double threshold = 11,
							bool force_uniformity = true);
	// Given an image and a set of initial features,  this function extracts and returns image 
	// features. New features are picked such that the resultant set of features are not close 
	// to eachother more than '_min_dist_between_feats' pixels and the number of features is 
	// not greater than '_max_num_feats'. Also, this function tries to evenly distrubute the 
	// features to the grid/patches.
	int extract_features(const cv::Mat &img, vector<Point2f> &features, const cv::Mat &mask = cv::Mat(0, 0, CV_8U));
	// This function sets the parameters that 'plot_features(...)' uses for visualization
	// radius := radius of the feature
	// thickness := Thickness of the circle. Negative values fills the circle
	// type := Pixel connectivity of the drawing function.
	int set_plotting_params(Scalar feat_color = Scalar(0,255,0), 
							Scalar gridcolor  = Scalar(200,200,200), 
							float radius = 3, float feat_thickness = -1, 
							float grid_thichness = 1, int type = 8);
	// This function plots the extracted features. 
	int plot_features(cv::Mat &img, vector<Point2f> &features, bool plot_gridlines = false);
	// Setter functions.
	void set_detector(string detector);
	void set_grid_size(Size2i grid_size){
		assert(grid_size.width > 0 && grid_size.height > 0);
		_grid_size = grid_size;
	}
	void set_min_dist(double min_dist){
		assert(min_dist > 0);
		_min_dist = min_dist;
	}
	void set_max_num_feats(int max_num_feats){
		assert(max_num_feats > 0);
		_max_num_feats = max_num_feats;
	}
	void set_threshold(double threshold){
		assert(threshold > 0);
		_threshold = threshold;
	}
	void modify_threshold(double delta_th){
		_threshold += delta_th;
		_threshold = _threshold >= 0 ? _threshold : 0.01;
	}
	void set_force_uniformity(bool force_uniformity){
		_force_uniformity = force_uniformity;
	}
};

#endif
