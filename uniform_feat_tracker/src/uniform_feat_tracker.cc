#include "uniform_feat_tracker.hh"
#include <numeric>

#include <ros/ros.h>

#define DEBUG 0
#if DEBUG
	#define debug_msg(A) {cout << "---- " << A << endl; fflush(NULL);}
#else
	#define debug_msg(A) {}
#endif


#define __timerstart {__tic = ros::Time::now();}
#define __timerstop(A)  {__toc = ros::Time::now(); debug_msg(A << " : " << (__toc - __tic).toSec())}

ros::Time __tic, __toc;

// ### I should put getter, setter functions for plotting params
// ### Resizable history and feature count

UniformFeatureTracker::UniformFeatureTracker(int max_num_feats, int max_hist_len){
	_max_num_feats = max_num_feats;
	_max_hist_len  = max_hist_len;
	_flow_thickness = 1.5;
	_use_aged_colors = false;
	_flow_color = Scalar(0, 0, 255);
	_feat_color = Scalar(0, 0, 255);
	_feat_radius = 2;
	_feat_thickness = -1;
	_aging_time = 3.3; // seconds
	_num_tracked_feats = 0;
	_max_flow_rate = 220;
	_num_stddevs = 3;
	_max_age = 300000; // Why?
	_curr_frame_idx = 0;
	_initialize();
}

int UniformFeatureTracker::_initialize(){
	_winSize = Size2i(21, 21);
	_max_pyra_level = 3;

	_frames.resize(_max_hist_len);
	_is_initialized.resize(_max_hist_len);
	_feat_history_grid.clear();
	_feat_history_grid.resize(_max_hist_len);
	for(int i = 0 ; i < _max_hist_len ; i++){
		_feat_history_grid[i].resize(_max_num_feats);
		_frames[i].resize(_max_pyra_level);
		_is_initialized[i] = false;
	}
	
	_last_feat_id = 0;
	_feat_ids.clear();
	_feat_ids.resize(_max_num_feats, -1);
	
	_start_frame_idx.clear();
	_start_frame_idx.resize(_max_num_feats, -1);
	_temp_feats.resize(_max_num_feats);

	_status.resize(_max_num_feats);
	_errs.resize(_max_num_feats);
	_feat_age.clear();
	_feat_age.resize(_max_num_feats, 0);
	
	_feat_colors.resize(_max_hist_len);
	//_feat_colors.resize(_aging_time*30);
	
	float r, g, b;
	for(int i = 0 ; i < (int)_feat_colors.size() ; i++){
		float mult = (float)i / _feat_colors.size();
		if(mult <= 0.33334){
			b = 255;
			g = 255 * (1 - 3*mult);
			r = 255 * (1 - 3*mult);
		} else if(mult <= 0.66667){
			b = 255 * (0.66667 - mult) * 3;
			g = 255 * (mult - 0.33337) * 3;
			r = 0;
		} else {
			b = 0;
			g = 255 * (1 - mult) * 3;
			r = 255 * (mult - 0.66667) * 3;;
		}
		_feat_colors[i] = Scalar(b, g, r);
	}
	
	
	_criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
	
	return 0;
}

int UniformFeatureTracker::track_features(const cv::Mat &img, UniformFeatureExtractor &unifFeatExt, const cv::Mat &mask){

	ASSERT(img.channels() == 1, "Image has to be single channel");
  ASSERT(mask.size() == cv::Size(0, 0) || mask.size() == img.size(), "Image and mask sizes has to be the same.")
  ASSERT(mask.channels() == 1, "Mask has to be a single channel matrix")

	_image_size.width  = img.cols;
	_image_size.height = img.rows;
	
	_prev_frame_idx = _curr_frame_idx;
	_curr_frame_idx = (_curr_frame_idx + 1) % _max_hist_len;

	debug_msg("A1")
	// Update _start_frame_idx for features survived >= _max_hist_len
	for(int i = 0 ; i < _max_num_feats ; i++)
		if(_start_frame_idx[i] == _curr_frame_idx)
				_start_frame_idx[i] = (_start_frame_idx[i] + 1) % _max_hist_len;

	// Generate and add the new image pyramid
	buildOpticalFlowPyramid(img, _frames[_curr_frame_idx], _winSize, _max_pyra_level);
	_is_initialized[_curr_frame_idx] = true;
	
	// ******************* TRACK EXSITING FEATURES ************************** //		
	// Do we have a 'previous frame'?
	debug_msg("A2")
  //cout << "_is_initialized.size() = " << _is_initialized.size() << endl;
	if(_is_initialized[_prev_frame_idx] == true){
		if(_num_tracked_feats != 0){
			debug_msg("A2.1")
			vector<Point2f> temp_feats_prev, temp_feats_curr;
			
			temp_feats_prev.reserve(_num_tracked_feats);
			temp_feats_curr.resize (_num_tracked_feats);
			for(int i = 0 ; i < _max_num_feats ; i++)
				if(_start_frame_idx[i] != -1)
					temp_feats_prev.push_back(_feat_history_grid[_prev_frame_idx][i]);
			debug_msg("A2.2")
    
			//ros::Time time_start, time_end;
			//time_start = ros::Time::now();
			calcOpticalFlowPyrLK(_frames[_prev_frame_idx], _frames[_curr_frame_idx], 
								 temp_feats_prev, temp_feats_curr, _status, _errs, 
								 _winSize, _max_pyra_level, _criteria);
			//time_end   = ros::Time::now();
			//cout << "calcOpticalFlow : " << (time_start - time_end).toSec() << endl;

			debug_msg("A2.3")
			for(int i = 0, j = 0 ; i < _max_num_feats ; i++){
				if(_start_frame_idx[i] != -1){
					debug_msg("A2.3.1")
					// If tracking is not successful, mark the features as non-tracked.
					if(_status[j] == 0){
						_start_frame_idx[i] = -1;
						_num_tracked_feats--;
					} else {
						// Update the feature pixel position for the current frame to the result of 
						// optical flow tracker.
						debug_msg("A2.3.2")
						_feat_history_grid[_curr_frame_idx][i] = temp_feats_curr[j];
						// For visualization and feature elimination
						_feat_age[i]++;
					}
					j++;
				}	
			}
			debug_msg("A2.4")
		}
	}

	debug_msg("A3")
	// Eliminate outliers using expected stability in the tracked feature locations.
	_eliminate_outliers(mask);

	// Done with tracking existing features. Elhm.
	// ********************** INITIALIZE NEW FEATURES ************************//
	// Extract features uniformly to fill up the empty slots
	static vector<Point2f> features;
	// If we still have room for new features, ask UniformFeatureExtractor for more.
	//cout << "_num_tracked_feats = " << _num_tracked_feats << endl;
	//cout << "_max_num_feats = " << _max_num_feats << endl;
	if(_num_tracked_feats != _max_num_feats){
		features.resize(_num_tracked_feats);
		for(int i = 0, j = 0 ; i < _max_num_feats ; i++)
			if(_start_frame_idx[i] != -1)
				features[j++]= _feat_history_grid[_curr_frame_idx][i];	
		//cout << "# of features before : " << features.size() << endl;
		unifFeatExt.extract_features(img, features, mask);
		//cout << "# of features after  : " << features.size() << endl;
	}

	debug_msg("A4")
	// Insert new features to the tracker
	if((int)features.size() > _num_tracked_feats){
		debug_msg("A4.1")
		int num_feats_to_add = features.size() - _num_tracked_feats;
		for(int i = 0 ; i < _max_num_feats && num_feats_to_add != 0 ; i++){
			debug_msg("A4.1.1")
			if(_start_frame_idx[i] == -1){
				debug_msg("A4.1.1.1")
				_feat_history_grid[_curr_frame_idx][i] = features[_num_tracked_feats + --num_feats_to_add];
				debug_msg("A4.1.1.2")
				_start_frame_idx[i] = _curr_frame_idx;
				_feat_age[i] = 0;
				debug_msg("A4.1.1.3")
				_feat_ids[i] = _last_feat_id++;
				debug_msg("A4.1.1.4")
			}
			debug_msg("A4.1.2")
		}
		_num_tracked_feats = features.size();;
		debug_msg("A4.2")
	}	

	debug_msg("A5")
	return 0;
}

int UniformFeatureTracker::plot_flow(cv::Mat &img, bool plot_flow, bool plot_feats) {
	Scalar &color = _flow_color;
	if(plot_flow == true){
		for(int i = 0 ; i < _max_num_feats ; i++) {
			if(_start_frame_idx[i] == -1)
				continue;
			
			for(int j = _start_frame_idx[i] ; j != _curr_frame_idx && _is_initialized[j] == true ; j = (j+1) % _max_hist_len){
				if(_use_aged_colors == true)
					color = _feat_age[i] < (int)_feat_colors.size() ? _feat_colors[_feat_age[i]] : _feat_colors.back();
				line(img, _feat_history_grid[j][i], 
						  _feat_history_grid[(j+1) % _max_hist_len][i], 
						  color, _flow_thickness);
			}
			/*
			if(plot_flow == 2 || true){
				Point2f pt1, pt2;
				pt1 = _feat_history_grid[_start_frame_idx[i]][i];
				pt2 = _feat_history_grid[_curr_frame_idx][i];
				line(img, pt1, pt2, _feat_colors[0], _flow_thickness);
				cout << "pt1 = " << pt1 << endl;
				cout << "pt2 = " << pt1 << endl;
			}
			*/
		}
	}
	
	if(plot_feats == true && _is_initialized[_curr_frame_idx]){
		color = _feat_color;
		for(int i = 0 ; i < _max_num_feats ; i++){
			if(_start_frame_idx[i] >= 0){
				if(_use_aged_colors == true)
					color = _feat_age[i] < (int)_feat_colors.size() ? _feat_colors[_feat_age[i]] : _feat_colors.back();
				circle(img, _feat_history_grid[_curr_frame_idx][i], _feat_radius, color, _feat_thickness);
			}
		}
	}
	
	return 0;
}

int UniformFeatureTracker::_eliminate_outliers(const cv::Mat &mask){
	// Also eliminate flow trails with unexpectedly large flow vectors
	bool mask_available = mask.cols * mask.rows != 0;
	for(int i = 0 ; i < _max_num_feats ; i++){
		if(_start_frame_idx[i] < 0 || _start_frame_idx[i] == _curr_frame_idx)
			continue;
		Point2f &pt_curr = _feat_history_grid[_curr_frame_idx][i];
		Point2f &pt_prev = _feat_history_grid[_prev_frame_idx][i];
		// If point is out of the image plane, remove it.
		if(pt_curr.x < 0 || pt_curr.y < 0 || pt_curr.x > _image_size.width || pt_curr.y > _image_size.height){
        _start_frame_idx[i] = -1;
        _num_tracked_feats--;
        continue;
		}
    // Stop tracking features which correspond to masked region
		if(mask_available == true && mask.at<char>(pt_curr.y, pt_curr.x) == 0){
		    _start_frame_idx[i] = -1;
        _num_tracked_feats--;
        continue;
    }

		if(fabs(pt_curr.x - pt_prev.x) > _max_flow_rate || fabs(pt_curr.y - pt_prev.y) > _max_flow_rate ||
			pt_curr.x < 0 || pt_curr.x >= _image_size.width ||
			pt_curr.y < 0 || pt_curr.y >= _image_size.height) {
			_start_frame_idx[i] = -1;
			_num_tracked_feats--;
		} else if(_feat_age[i] > _max_age) {
			_start_frame_idx[i] = -1;
			_num_tracked_feats--;
		}
	}
	
	// Calculate the total length of trails. Expect all of them not too larger
	// than the mean length. 
	vector<float> flow_lengths;
	flow_lengths.reserve(_max_num_feats);
	for(int i = 0 ; i < _max_num_feats ; i++) {
		int init_frame_idx = _start_frame_idx[i];
		if(init_frame_idx < 0)
			continue;
		float len = 0;
		int num_segments = 0;
		for(int j = init_frame_idx ; j != _curr_frame_idx && _is_initialized[j] == true ; j = (j+1) % _max_hist_len){
			Point2f &pt1 = _feat_history_grid[j][i];
			Point2f &pt2 = _feat_history_grid[(j+1) % _max_hist_len][i];
			len += sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
			num_segments++;
		}
		len /= num_segments;
		flow_lengths.push_back(len);
	}
	
	double flow_sum = std::accumulate(flow_lengths.begin(), flow_lengths.end(), 0.0);
	double flow_mean = flow_sum / flow_lengths.size();

	double sq_sum = std::inner_product(flow_lengths.begin(), flow_lengths.end(), flow_lengths.begin(), 0.0);
	double stdev = std::sqrt(sq_sum / flow_lengths.size() - flow_mean * flow_mean);
	
	for(int i = 0, j = 0 ; i < _max_num_feats ; i++) {
		if(_start_frame_idx[i] < 0)
			continue;
		if(flow_lengths[j] - flow_mean >= _num_stddevs * stdev) {
			_start_frame_idx[i] = -1;
			_num_tracked_feats--;
		}
		j++;
	}
	
	return 0;
}

// ### This function needs more thinking!!!
// ### This may need more work. I will see insh. what I need when I progress 
// in mapping (maybe use bundle adjustment)
int UniformFeatureTracker::get_features(vector<Point2d> &features, vector<int> &feat_ids, int lifetime){
	if((int)feat_ids.size() != _max_num_feats)
		feat_ids.resize(_max_num_feats, false);
	
	if(lifetime < 0 || lifetime >= _max_hist_len)
		lifetime = _max_hist_len - 1;
	int frame_idx = (_curr_frame_idx - lifetime + _max_hist_len) % _max_hist_len;
	
	features.resize(_max_num_feats);
	for(int i = 0 ; i < _max_num_feats ; i++){
		features[i].x = _feat_history_grid[frame_idx][i].x;
		features[i].y = _feat_history_grid[frame_idx][i].y;
	}
	
	int retval = 0;
	for(int i = 0 ; i < _max_num_feats ; i++){
		if((_feat_age[i] >= lifetime) && (_start_frame_idx[i] >= 0)){
			feat_ids[i] = _feat_ids[i];
			retval++;
		} else
			feat_ids[i] = -1;
	}
	
	return retval;
}

int UniformFeatureTracker::get_features(vector<Point2f> &features, vector<int> &feat_ids, int lifetime){
	if((int)feat_ids.size() != _max_num_feats)
		feat_ids.resize(_max_num_feats, false);
	
	if(lifetime < 0 || lifetime >= _max_hist_len)
		lifetime = _max_hist_len - 1;
	int frame_idx = (_curr_frame_idx - lifetime + _max_hist_len) % _max_hist_len;
	
	features = _feat_history_grid[frame_idx];
	
	int retval = 0;
	for(int i = 0 ; i < _max_num_feats ; i++){
		if((_feat_age[i] >= lifetime) && (_start_frame_idx[i] >= 0)){
			feat_ids[i] = _feat_ids[i];
			retval++;
		} else
			feat_ids[i] = -1;
	}
		
	return retval;
}

#undef __timerstart
#undef __timerstop
#undef DEBUG_FLAG
#undef debug_msg




