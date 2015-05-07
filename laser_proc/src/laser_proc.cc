#include "laser_proc.hh"

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

/*
class LaserProc{
  private:
    // This class can handle multiple scan at once. The original
    // data is saved in the '_orig_scans' vector
    sensor_msgs::LaserScan _data;       // saves the scan data. Overwritten if required.
    LidarCalibParams _params;           // saves a copy of the parameter set.
    vector<Eigen::Vector3d> _3d_rays;   // saves the transformed 3D point set.
    vector<Eigen::Vector2d> _2d_rays;   // saves the projected 2D point set.
    vector<int> _mask;                  // encodes validity of range data saved in '_data'.
    // Also saves the cluster id's.
    vector<double> _lineartiy;          // encodes whether a point belongs to a line of corner.
    vector<pair<int, int> > _line_idxs; // saves the indices of the end points of lines.

    // This stores a set of distinctive colors for RVIZ visualization.
    vector<Eigen::Vector3i> _colors;
    
  public:

*/

void LaserProc::_initialize(){
	// Resize the containers in case the new scan data has
	// different number of ranges.
	_3d_rays.resize(_ranges.size());
	_2d_rays.resize(_ranges.size());
	_linearity.resize(_ranges.size());
	_line_idxs.resize(_ranges.size());
	_mask.resize(_ranges.size());

	std::fill(_mask.begin(), _mask.end(), 1);

	// Mask the regions to be omitted (dead_regions)    
    for(int i = 0 ; i < (int)_params.dead_regions.size() ; i+=2){
        double th_begin = _params.dead_regions[i];
        double th_end   = _params.dead_regions[i+1];
        int idx_begin = std::max(0.0, 
                round((th_begin - _angle_min) / _angle_increment));
        int idx_end   = std::min(_ranges.size() - 1.0, 
                round((th_end   - _angle_min) / _angle_increment));
        for(int j = idx_begin ; j <= idx_end ; j++)
            _mask[j] = 0;
    } 

	// Mask the regions with bad range values as invalid 
	for(int i = 0 ; i < (int)_mask.size() ; i++){
		double range = _ranges[i];
		if(range != range || range < _params.min_range || 
			range > _params.max_range || !isfinite(range))
			_mask[i] = 0;
	}
}

LaserProc::LaserProc(){ }

const vector<int>& LaserProc::update_data(const sensor_msgs::LaserScan &data, const LidarCalibParams &params){
	_angle_min = data.angle_min;
	_angle_max = data.angle_max;
	_angle_increment = data.angle_increment;
	_ranges = vector<double>(data.ranges.begin(), data.ranges.end());
	_params = params;
	_initialize();
	return _mask;
}
    
const vector<int>& LaserProc::update_data(const sensor_msgs::LaserScan &data){
	_angle_min = data.angle_min;
	_angle_max = data.angle_max;
	_angle_increment = data.angle_increment;
	_ranges = vector<double>(data.ranges.begin(), data.ranges.end());

	_initialize();
	return _mask;
}
    
const vector<double>& LaserProc::median_filter(int win_size){
	ASSERT(win_size > 0, "'win_size > 0' has to hold");

	int num_ranges = _ranges.size();
	_temp_ranges.resize(num_ranges);

	vector<double> buffer(2 * win_size + 1);

	for(int i = 0 ; i < num_ranges ; i++){
		if(_mask[i] == false){
			// Even though the values is not valid, just save it.
			_temp_ranges[i] = _ranges[i];
			continue;
		}
		// # of valid points in the buffer
		int valid_pts = 0;

		for(int j = i - win_size, k = 0 ; j <= i + win_size ; j++, k++){
			if(j < 0 || j >= num_ranges || _mask[j] == false)
				buffer[k] = -1;
			else {
				buffer[k] = _ranges[j];
				valid_pts++;
			}
		}
	
		// Update the median according to the below logic.
		double &median = _temp_ranges[i];
		if(valid_pts == 1)
			median = _ranges[i];
		else {
			// Sort the ranges in descending order. This will push the
			// '-1's to the end of the vector.
			std::sort(buffer.begin(), buffer.end(), std::greater<double>());
			if(valid_pts % 2 == 1)
				median = buffer[(valid_pts - 1) / 2];
			else
				median = (buffer[valid_pts / 2 - 1] + buffer[valid_pts / 2]) / 2;
		}
	}

	_ranges = _temp_ranges;
	return _ranges;
}
    
const vector<int>& LaserProc::downsample(double range_thres){
	ASSERT(range_thres > 0, "(range_thres > 0) should hold.")

	int num_ranges = _ranges.size();
	for(int i = 0 ; i < num_ranges ; i++){
		if(_mask[i] == false)
			continue;
		double range1 = _ranges[i];
		for(int j = i + 1 ; j < num_ranges && j < i + 13 ; j++){
			if(_mask[j] == false)
				continue;
			double range2 = _ranges[j];
			double gamma = (j - i) * _angle_increment;
			double d = sqrt(range1 * range1 + range2 * range2 - 2 * range1 * range2 * cos(gamma));
			if(d < range_thres)
				_mask[j] = false;
			else
				break;
		}
	}

	return _mask;
}
    
const vector<int>& LaserProc::remove_shadows(double range_thres){
	ASSERT(range_thres > 0, "(range_thres > 0) must hold.")
	int num_ranges = _ranges.size();
	
	for(int i = 0 ; i < num_ranges - 1; i++){
		if(_mask[i] == false || _mask[i+1] == false)
			continue;
		double range1 = _ranges[i];
		double range2 = _ranges[i + 1];
		if(range1 - range2 > range_thres)
			_mask[i] = false;
		else if(range2 - range1 > range_thres)
			_mask[i + 1] = false;
	}

	return _mask;
}
    
const vector<int>& LaserProc::clusters(int min_skips, double range_jump, int min_cluster_size){
	return _mask;
}
    
const vector<double>& LaserProc::rate_linearity(int win_size){
	return _linearity;
}
    
const vector<pair<int, int> >& LaserProc::extract_lines(double min_len, double min_pts, double epsilon){
	return _line_idxs; 
}
    
const vector<Eigen::Vector3d>& LaserProc::transform(const Eigen::Matrix4d &trans, bool is_sensor_pose){
	return _3d_rays;
}
    
const vector<Eigen::Vector2d>& LaserProc::project(const Eigen::Vector3d &n){
	return _2d_rays;
}
    
const vector<double>& LaserProc::get_ranges(){
	return _ranges;
}
    
const Eigen::Matrix3d& LaserProc::get_fim(){
	return _fim;
}
    
const vector<int>& LaserProc::get_mask_array(){
	return _mask;
}
    
const vector<Eigen::Vector3d>& LaserProc::get_3d_points(){
	return _3d_rays;
}
    
const vector<double>& LaserProc::get_linearity_rates(){
	return _linearity;
}
    
const vector<Eigen::Vector2d>& LaserProc::get_projections(){
	return _2d_rays;
}
    
bool LaserProc::get_RVIZ_markers(visualization_msgs::MarkerArray &marray){
	return true;
}




