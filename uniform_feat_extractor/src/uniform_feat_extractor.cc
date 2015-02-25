#include "uniform_feat_extractor.hh"

//#define debug_msg(A) {cout << "---- " << A << endl;}
#define debug_msg(A) {}

bool keypoint_sort(const KeyPoint &kp1, const KeyPoint &kp2){return kp1.response > kp2.response;}

bool UniformFeatureExtractor::_profile_grids(const vector<Point2f> &features){
	if(_histogram.cols != _grid_size.width ||
		_histogram.rows != _grid_size.height)
		_histogram = cv::Mat::zeros(_grid_size.height, _grid_size.width, CV_32SC1);
	else
		_histogram.setTo(Scalar(0));

	int r, c, num_feats = features.size();
	for(int i = 0 ; i < num_feats ; i++){
		r = features[i].y / _patch_size.height;
		c = features[i].x / _patch_size.width;
		r = r >= (int)_histogram.rows ? _histogram.rows - 1 : r;
		c = c >= (int)_histogram.cols ? _histogram.cols - 1 : c;
		_histogram.at<int>(r, c)++;
	}

	return true;
}
	
// ### Define a structure to pass as reference. Individual parameters cause burden
int UniformFeatureExtractor::set_plotting_params(Scalar feat_color, Scalar grid_color, 
												 float radius, float feat_thickness, 
												 float grid_thickness, int type){
	_feat_color     = feat_color;
	_grid_color     = grid_color;
	_radius         = radius;
	_feat_thickness = feat_thickness;
	_grid_thickness = grid_thickness;
	_type           = type;
	return 0;
}

// ### shi tomasi parameters should be settable from function call. May be write setter, getter functions.
UniformFeatureExtractor::UniformFeatureExtractor(string detector_type, double min_dist_between_feats,
												int max_num_feats, Size2i grid_size, double threshold,
												bool force_uniformity){
	_min_dist = min_dist_between_feats;
	_max_num_feats = max_num_feats;
	_grid_size = grid_size;
	_threshold = threshold;
	_threshold = _threshold;
	_force_uniformity = force_uniformity;
	_detector_type = detector_type;
	std::transform(_detector_type.begin(), _detector_type.end(), _detector_type.begin(), ::tolower);
	set_detector(detector_type);
	set_plotting_params();
}

int UniformFeatureExtractor::extract_features(const cv::Mat &img, vector<Point2f> &features, const cv::Mat &mask){
  ASSERT(mask.size() == cv::Size(0, 0) || mask.size() == img.size(), "mask and image must have same sizes")

	features.reserve(_max_num_feats);
	static vector<KeyPoint> keypoints;

	_patch_size.width  = (float)img.cols / _grid_size.width;
	_patch_size.height = (float)img.rows / _grid_size.height;

	debug_msg("B1")
	// Prepare mask. OpenCV expects points of interest to be marked with
	// some non-zero value. We first check if _mask is already allocated.
	// Then set to a non-zero value by default.
	if(mask.size() != cv::Size(0, 0))
		mask.copyTo(_mask);
	else if(_mask.cols != img.cols || _mask.rows != img.rows)
		_mask = cv::Mat::ones(img.rows, img.cols, CV_8U);
	else
		_mask.setTo(Scalar(1));
	for(int i = 0 ; i < (int)features.size() ; i++)
		// Paint circles centered at each feature with '_min_dist' radius
		circle(_mask, features[i], _min_dist, 0, -1);

	debug_msg("B2")
	// According to the detector type, call the 'detect(...)' function.
	// Harris detector has the nice property of settin the maximum number of 
	// features to be extracted whereas FAST does not. Use _mask to load
	// the tedious vicinity check to OpenCV.
	bool is_fast = false;
	if(_detector_type == "fast"){
		_fast_detector.detect(img, keypoints, _mask);
		is_fast = true;
	} else {
		_gftt_detector.detect(img, keypoints, _mask);
	}

	debug_msg("B4")

	// Extracted features are sorted in order to first add stronger 
	// corners to the output list of features. 'keypoint_sort(...)'
	// function (defined at the top of the file) is used as predicate.
	std::sort(keypoints.begin(), keypoints.end(), keypoint_sort);

	debug_msg("B5")
	// Add the strongest 13 of the keypoints unconditionally. 
	// (*) Then starting from the patch which has the least features, 
	// add new keypoints to the output list.
	for(int i = 0 ; i < 13 && (int)features.size() < _max_num_feats ; i++){
	  if(_mask.at<char>(keypoints[i].pt.y, keypoints[i].pt.x) == 0)
	      continue;
		features.push_back(keypoints[i].pt);
		if(is_fast)
			circle(_mask, keypoints[i].pt, _min_dist, 0, -1);
	}

	
	debug_msg("B6")

	if(_force_uniformity == false){
		for(int i = 13 ; i < (int)keypoints.size() && (int)features.size() < _max_num_feats ; i++){
			if(is_fast == false) {
				features.push_back(keypoints[i].pt);
			} else {
				Point2f pt = keypoints[i].pt;
				if(_mask.at<char>(pt.y, pt.x) == 0)
					continue;
				else {
					features.push_back(pt);
					circle(_mask, pt, _min_dist, 0, -1);
				}
			}
		}

		return features.size();
	}

	
	debug_msg("B7")

	if((int)features.size() >= _max_num_feats) 
		return _max_num_feats;

	_profile_grids(features);
	
	debug_msg("B8")

	// Buffers to hold row and column indices of keypoints
	static vector<int> rs, cs;
	rs.resize(keypoints.size());
	cs.resize(keypoints.size());
	// rs[...] is used as an array of flags to define : 'unprocessed = -2' and
	// 'invalid = -1' keypoints. Keypoints whose grid indices are not determined
	// are called 'unprocessed'. Those processed but either used or failed mask
	// test are marked as 'invalid'. For performance issues we don't preprocess 
	// all keypoints.
	std::fill(rs.begin(), rs.end(), -2);
	int skip_grid_flag = 999999;
	while((int)features.size() < _max_num_feats){
		debug_msg("B8.1")
		// Indices of the grid cell with the lowest # of features.
		// Keypoint with the highest response that fall into the above
		// defined cell is sought.
		Point2i ind;
		double  val;
		minMaxLoc(_histogram, &val, NULL, &ind, NULL);
		debug_msg("B8.1A")
		// minMaxLoc hitting a cell with the skip value shows that
		// all cells has been traversed although output features list
		// could not be populated to its maximum value.
		if(val == skip_grid_flag)
			break;
		debug_msg("B8.2")
		// Search for the aforementioned keypoint.
		bool has_more_keypts = false;
		for(int i = 13 ; i < (int)keypoints.size() ; i++){
			//debug_msg("B8.2.1")
			if(rs[i] == -1)
				continue;
			Point2f pt = keypoints[i].pt;
			// Check if row and col index of this keypoint has been calculated
			if(rs[i] == -2){
				if(is_fast == true &&  _mask.at<char>(pt.y, pt.x) == 0){
					rs[i] = -1;
					continue;
				}
				rs[i] = pt.y / _patch_size.height;
				cs[i] = pt.x / _patch_size.width;
				rs[i] = rs[i] >= _histogram.rows ? _histogram.rows - 1 : rs[i];
				cs[i] = cs[i] >= _histogram.cols ? _histogram.cols - 1 : cs[i];
			}
			
			//debug_msg("B8.2.2")
			if(rs[i] == ind.y && cs[i] == ind.x){
				rs[i] = -1; // invalidate current keypoint
				has_more_keypts = true;
				features.push_back(pt);
				_histogram.at<int>(ind.y, ind.x)++;					
				if(is_fast == true)
					circle(_mask, pt, _min_dist, 0, -1);
				break;
			}
		}

		debug_msg("B8.3")
		// If no change has happened, keypoints do not have anymore 
		// points falling into the current grid cell. Assign a random 
		// large value to skip it in the next iteration.
		if(has_more_keypts == false)
			_histogram.at<int>(ind.y, ind.x) = skip_grid_flag;
	}

	debug_msg("B9")
	
	return 0;		
}

int UniformFeatureExtractor::plot_features(cv::Mat &img, vector<Point2f> &features, bool plot_gridlines){
	if(plot_gridlines)	{	
		for(int c = 0 ; c <= _grid_size.width ; c++)
			line(img, Point2f(c * _patch_size.width, 0), Point2f(c * _patch_size.width, img.rows), _grid_color, _grid_thickness);
		for(int r = 0 ; r <= _grid_size.height ; r++)
			line(img, Point2f(0, r * _patch_size.height), Point2f(img.cols, r * _patch_size.height), _grid_color, _grid_thickness);
	}

	for(int i = 0 ; i < (int)features.size() ; i++)
		circle(img, features[i], _radius, _feat_color, _feat_thickness, _type);

	
	return 0;
}

void UniformFeatureExtractor::set_detector(string detector_type){
	std::transform(detector_type.begin(), detector_type.end(), detector_type.begin(), ::tolower);

	if(detector_type == "fast")
		_fast_detector = FastFeatureDetector(_threshold);
	else if(detector_type == "harris")
		_gftt_detector = GoodFeaturesToTrackDetector(_max_num_feats, _threshold, _min_dist, 3, true);
	else if(detector_type == "gftt")
		_gftt_detector = GoodFeaturesToTrackDetector(_max_num_feats, _threshold, _min_dist);
	else {
		cv::Exception ex;
		ex.code = 0;
		ex.err = "'detector_type' should be one of 'Harris', 'GFTT', 'FAST'";
		ex.func = string(__func__);
		ex.file = string(__FILE__);
		ex.line = __LINE__;
		throw ex;
	}

	_detector_type = detector_type;
}

#undef debug_msg

