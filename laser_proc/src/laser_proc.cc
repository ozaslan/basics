#include "laser_proc.hh"

void LaserProc::_initialize(){
  // Resize the containers in case the new scan data has
  // different number of ranges.
  _linearity.resize(_ranges.size());
  _line_idxs.resize(_ranges.size());
  _mask.resize(_ranges.size());

  // Set cluster id's for all the points as '1'
  std::fill(_mask.begin(), _mask.end(), 1);

  _num_clusters = 2; // we are sure that we have '0', '1' clusters.

  if(_thetas.size() != _ranges.size() ||
      _thetas[0] != _angle_min || _thetas.back() != _angle_max){
    _thetas.resize(_ranges.size());
    for(int i = 0 ; i < (int)_thetas.size() ; i++)
      _thetas[i] = _angle_min + i * _angle_increment;
  }

  // Mask the regions to be omitted (dead_regions) (invalid = {_mask[i] = 0})
  for(int i = 0 ; i < (int)_params.dead_regions.size() ; i+=2){
    double th_begin = _params.dead_regions[i];
    double th_end   = _params.dead_regions[i+1];
    int idx_begin = std::max(0.0, 
        round((th_begin - _angle_min) / _angle_increment));
    int idx_end   = std::min(_ranges.size() - 1.0, 
        round((th_end   - _angle_min) / _angle_increment));
    for(int j = idx_begin ; j <= idx_end ; j++)
      _mask[j] = false;
  } 

  for(int i = 0 ; i < (int)_params.upwards_mirror_regions.size() ; i+=2){
    double th_begin = _params.upwards_mirror_regions[i];
    double th_end   = _params.upwards_mirror_regions[i+1];
    int idx_begin = std::max(0.0, 
        round((th_begin - _angle_min) / _angle_increment));
    int idx_end   = std::min(_ranges.size() - 1.0, 
        round((th_end   - _angle_min) / _angle_increment));
    for(int j = idx_begin ; j <= idx_end ; j++)
      _mask[j] = -1;
  } 

  for(int i = 0 ; i < (int)_params.downwards_mirror_regions.size() ; i+=2){
    double th_begin = _params.downwards_mirror_regions[i];
    double th_end   = _params.downwards_mirror_regions[i+1];
    int idx_begin = std::max(0.0, 
        round((th_begin - _angle_min) / _angle_increment));
    int idx_end   = std::min(_ranges.size() - 1.0, 
        round((th_end   - _angle_min) / _angle_increment));
    for(int j = idx_begin ; j <= idx_end ; j++)
      _mask[j] = -2;
  } 

  // Mask the regions with bad range values as invalid (_mask[i] = 0)
  for(int i = 0 ; i < (int)_mask.size() ; i++){
    double& range = _ranges[i];
    if(range != range || range < _params.min_range || 
        range > _params.max_range || !isfinite(range))
      _mask[i] = false;
  }

  _generate_colors();

  _2d_euclidean_coords_valid = false;
  _3d_euclidean_coords_valid = false;
}

void LaserProc::_polar_to_2d_euclidean(){
  if(_2d_euclidean_coords_valid == true)
    return;

  int num_ranges = _ranges.size();
  _2d_rays.resize(num_ranges);
  for(int i = 0 ; i < num_ranges ; i++){
    double th = _angle_min + i * _angle_increment;
    _2d_rays[i](0) = _ranges[i] * cos(th);                                      
    _2d_rays[i](1) = _ranges[i] * sin(th);
  }

  _2d_euclidean_coords_valid = true;
}

void LaserProc::_polar_to_3d_euclidean(){
  int num_ranges = _ranges.size();
  _3d_rays.resize(num_ranges);
  for(int i = 0 ; i < num_ranges ; i++){
    double th = _angle_min + i * _angle_increment;
    _3d_rays[i](0) = _ranges[i] * cos(th);                                      
    _3d_rays[i](1) = _ranges[i] * sin(th);
    _3d_rays[i](2) = 0;
    if(_mask[i] == -1){
      _3d_rays[i](2) = _3d_rays[i].norm();
      _3d_rays[i](0) = _3d_rays[i](1) = 0;
    } else if (_mask[i] == -2){
      _3d_rays[i](2) = -_3d_rays[i].norm();
      _3d_rays[i](0) = _3d_rays[i](1) = 0;
    }
  }

  _3d_euclidean_coords_valid = true;
}


LaserProc::LaserProc(){ }

const vector<int>& LaserProc::update_data(const sensor_msgs::LaserScan &data, const LidarCalibParams &params){
  _angle_min = data.angle_min;
  _angle_max = data.angle_max;
  _angle_increment = data.angle_increment;
  _ranges = vector<double>(data.ranges.begin(), data.ranges.end());
  _intensities = vector<double>(data.intensities.begin(), data.intensities.end());
  _sec = data.header.stamp.sec;
  _nsec = data.header.stamp.nsec;
  _params = params;
  _2d_euclidean_coords_valid = false;
  _3d_euclidean_coords_valid = false;

  _initialize();
  return _mask;
}

const vector<int>& LaserProc::update_data(const sensor_msgs::LaserScan &data){
  _angle_min = data.angle_min;
  _angle_max = data.angle_max;
  _angle_increment = data.angle_increment;
  _sec = data.header.stamp.sec;
  _nsec = data.header.stamp.nsec;
  _ranges = vector<double>(data.ranges.begin(), data.ranges.end());
  _intensities = vector<double>(data.intensities.begin(), data.intensities.end());
  _2d_euclidean_coords_valid = false;
  _3d_euclidean_coords_valid = false;

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

    // If the ray is not one of the up/downward looking rays
    if(_mask[i] > 0){
      for(int j = i - win_size, k = 0 ; j <= i + win_size ; j++, k++){
        if(j < 0 || j >= num_ranges || _mask[j] < 0)
          buffer[k] = -1;
        else {
          buffer[k] = _ranges[j];
          valid_pts++;
        }
      }
    } else {
      // If the ray is one of up/downward facing rays.
      for(int j = i - win_size, k = 0 ; j <= i + win_size ; j++, k++){
        if(j < 0 || j >= num_ranges || _mask[j] != _mask[i])
          buffer[k] = -1;
        else {
          buffer[k] = _ranges[j];
          valid_pts++;
        }
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

  _2d_euclidean_coords_valid = false;
  _3d_euclidean_coords_valid = false;

  return _ranges;
}

const vector<int>& LaserProc::intensity_filter(double intensity_thres){
  int num_ranges = _mask.size();
  for(int i = 0 ; i < num_ranges ; i++){
    _mask[i] = _intensities[i] < intensity_thres ? 0 : _mask[i];
  }

  return _mask;
}

const vector<int>& LaserProc::linearity_filter(double linearity_thres, bool upper_bound){
  ASSERT(_linearity.size() == _mask.size(), "_linearity.size() == _mask.size()");

  int num_ranges = _mask.size();
  if(upper_bound == true){
    for(int i = 0 ; i < num_ranges ; i++){
      _mask[i] = _linearity[i] > linearity_thres ? 0 : _mask[i];
    }
  } else {
    for(int i = 0 ; i < num_ranges ; i++){
      _mask[i] = _linearity[i] < linearity_thres ? 0 : _mask[i];
    }
  }

  return _mask;
}


const vector<int>& LaserProc::downsample(double range_thres){
  ASSERT(range_thres > 0, "(range_thres > 0) should hold.");

  int num_ranges = _ranges.size();
  for(int i = 0 ; i < num_ranges ; i++){
    // Exclude invalid and up/downward facing rays.
    if(_mask[i] <= 0)
      continue;
    double range1 = _ranges[i];
    for(int j = i + 1 ; j < num_ranges ; j++){
      if(_mask[j] <= 0)
        continue;
      double range2 = _ranges[j];
      // Even if the next ray might be closer than range_thres,
      // clustering will put these two rays in different
      // buckets. Thus, the loop can be terminated.
      if(fabs(range2 - range1) > range_thres){
        break;
      }

      double gamma = (j - i) * _angle_increment;
      double d = range1 * range1 + range2 * range2 - 2 * range1 * range2 * cos(gamma);
      if(d < range_thres * range_thres){
        _mask[j] = 0;
      } else {
        break;
      }
    }
  }
  return _mask;
}

const vector<int>& LaserProc::remove_slant_edges(double angle_thres, int win_size){
  angle_thres = fabs(utils::fix_angle(angle_thres));

  ASSERT(fabs(angle_thres) < PI/2, "fabs(angle_thres) < PI / 2 ");

  angle_thres = PI / 2 - angle_thres;

  if(_2d_euclidean_coords_valid == false)
    _polar_to_2d_euclidean();

  int num_ranges = _ranges.size();

  vector<int> rays_to_remove;
  rays_to_remove.reserve(num_ranges / 50);

  Eigen::Vector2d pt1, pt2, d;
  double angle;
  for(int i = 1 ; i < num_ranges - 1; i++){
    if(_mask[i] <= 0 || _mask[i - 1] <= 0 || _mask[i + 1] <= 0)
      continue;
    pt1 = _2d_rays[i - 1];
    pt2 = _2d_rays[i + 1];
    d   = pt2 - pt1;
    d  /= d.norm();
    angle = acos(d.dot(_2d_rays[i]) / _2d_rays[i].norm());
    // If the angle between the two rays is very small, then the ray is invalid.
    if(angle < angle_thres)
      rays_to_remove.push_back(i);
  }

  for(int i = 0 ; i < (int)rays_to_remove.size() ; i++){
    for(int j = i - win_size ; j <= i + win_size ; j++)
      if(j >= 0 && j < num_ranges && _mask[j] > 0)
        _mask[j] = false;
  }

  return _mask;
}

const vector<int>& LaserProc::remove_occlusions(double range_thres, int win_size){
  ASSERT(range_thres > 0, "(range_thres > 0) must hold.");
  int num_ranges = _ranges.size();

  for(int i = 0 ; i < num_ranges - 1; i++){
    if(_mask[i] <= 0 || _mask[i+1] <= 0)
      continue;
    double range1 = _ranges[i];
    double range2 = _ranges[i + 1];
    if(range1 - range2 > range_thres){
      // Don't invalidate up/downward looking rays
      for(int j = i ; j >= 0 && j > i - win_size; j--)
        _mask[j] = _mask[j] > 0 ? false : _mask[j];
    } else if(range2 - range1 > range_thres){
      for(int j = i + 1 ; j < num_ranges && j < i + 1 + win_size; j++)
        _mask[j] = _mask[j] > 0 ? false : _mask[j];
    }
  }

  return _mask;
}

const vector<int>& LaserProc::cluster(int min_skips, double range_jump, int min_cluster_size){
  int num_ranges = _ranges.size();
  int current_label = 1;
  int current_cluster_size = 0;
  int skipped_rays = 0;
  double prev_range, range;
  prev_range = -999;

  _num_clusters = 1; // we are sure that we have '0' cluster.

  for(int i = 0 ; i < num_ranges ; i++){
    // Skip up/downward looking rays
    if(_mask[i] <= 0){
      skipped_rays++;
      continue;
    }

    range = _ranges[i];

    // Should a new cluster be initialized?
    // If positive, then also check the size of the previous cluster
    // and delete if the number of rays in it is smaller than the
    // minimum allowable number of rays
    if(skipped_rays >= min_skips || fabs(range - prev_range) > range_jump){
      // Before initializing a new cluster, check 
      // the size of the previous one. If the size is
      // small, delete it.
      if(current_cluster_size < min_cluster_size){
        for(int j = i - 1 ; j >= 0 ; j--){
          if(_mask[j] < 0)
            continue;
          else if(_mask[j] == current_label)
            _mask[j] = 0;
          else if(_mask[j] == current_label - 1)
            break;
        }
        current_label--;
      }
      // Initialize a new cluster.
      current_cluster_size = 0;   
      current_label++;
    }

    // Reset skipped_rays
    skipped_rays = 0;
    current_cluster_size++;
    prev_range = range;
    _mask[i] = current_label;
  } 

  // Before returning check again the total number of
  // rays in the last cluster. Delete if it is too small.
  if(current_cluster_size < min_cluster_size){
    for(int j = num_ranges - 1 ; j >= 0 ; j--){
      if(_mask[j] < 0)
        continue;
      if(_mask[j] == current_label)
        _mask[j] = 0;
      else if(_mask[j] == current_label - 1)
        break;
    }
    current_label--;
  } 

  _num_clusters = current_label + 1;

  return _mask;
}

const vector<double>& LaserProc::rate_linearity(int win_size){
  int num_ranges = _ranges.size();

  // First do transform coordinates from polar to Euclidean
  if(_2d_euclidean_coords_valid == false)
    _polar_to_2d_euclidean();

  for(int i = 0 ; i < num_ranges ; i++){
    if(_mask[i] < 0){
      continue;
    }
    Eigen::Vector2d temp(0, 0);
    int num_valid_pts = 0;
    for(int j = i - win_size ; j < i + win_size ; j++){
      if(i == j || j < 0 || j >= num_ranges || _mask[j] < 0)
        continue;
      temp += _2d_rays[j] - _2d_rays[i];
      num_valid_pts++;
    }
    _linearity[i] = temp.norm() / num_valid_pts / _2d_rays[i].norm();
  }

  return _linearity;
}

const vector<pair<int, int> >& LaserProc::extract_lines(double min_len, double min_pts, double epsilon){
  ASSERT(min_len > 0 && min_pts > 0 && epsilon > 0, "All arguments should be greater than zero.");

  int num_ranges = _ranges.size();

  _polar_to_2d_euclidean();

  vector<int> corner_idxs;
  vector<cv::Point2f> raw_pts, poly_pts;

  _line_idxs.clear();

  int start_idx = 0 , end_idx = 0;
  while(start_idx < num_ranges){
    // start_idx : inclusive, end_idx : exclusive
    for(                        ; start_idx < num_ranges && _mask[start_idx] <= 0 ; start_idx++);
    for(end_idx = start_idx + 1 ;   end_idx < num_ranges && _mask[  end_idx] >  0 ;   end_idx++);

    // Prepare the input to cv::approxPolyDP(...)
    raw_pts.clear();
    raw_pts.reserve(end_idx - start_idx);
    for(int i = start_idx ; i < end_idx ; i++)
      raw_pts.push_back(cv::Point2f(_2d_rays[i](0), _2d_rays[i](1)));

    // Fit lines using OpenCV API.
    cv::approxPolyDP(raw_pts, poly_pts, epsilon, false);

    // Find the correspondences between the _ranges and poly_pts
    corner_idxs.clear();
    corner_idxs.reserve(poly_pts.size());
    for(int i = start_idx, r = 0, p = 0 ; i < end_idx && 
        p < (int)poly_pts.size() && 
        r < (int)raw_pts.size() ; i++){
      if(raw_pts[r++] == poly_pts[p]){
        corner_idxs.push_back(i);
        p++;
      }
    }

    // Add line idxs to _line_idxs vector
    for(int i = 0 ; i < (int)corner_idxs.size() - 1; i++){
      int idx1, idx2;
      idx1 = corner_idxs[i];
      idx2 = corner_idxs[i + 1];
      if(idx2 - idx1 + 1 >= min_pts){
        cv::Point2d diff = poly_pts[idx2] - poly_pts[idx1];
        if(diff.dot(diff) >= min_len * min_len)
          _line_idxs.push_back(make_pair<int, int>(idx1, idx2));
      }
    }

    start_idx = end_idx;
  }

  return _line_idxs; 
}

const vector<Eigen::Vector3d>& LaserProc::transform(const Eigen::Matrix4d &trans, bool is_sensor_pose){

  _polar_to_3d_euclidean();

  int num_ranges = _3d_rays.size();

  Eigen::Matrix4d temp_trans;

  if(is_sensor_pose == false)
    temp_trans = trans * _params.relative_pose;
  else
    temp_trans = trans;

  //if(temp_trans == Eigen::Matrix4d::Identity())
  //	return _3d_rays;

  const Eigen::Matrix3d dcm = temp_trans.topLeftCorner<3, 3>();
  const Eigen::Vector3d t   = temp_trans.topRightCorner<3, 1>();

  for(int i = 0 ; i < num_ranges ; i++){
    _3d_rays[i] = dcm * _3d_rays[i] + t;
  }

  return _3d_rays;
}

const vector<Eigen::Vector3d>& LaserProc::project(const Eigen::Vector3d &n){

  //if(_3d_euclidean_coords_valid == false)
  //  _polar_to_3d_euclidean();

  Eigen::Vector3d temp_n = n / n.norm();

  int num_ranges = _ranges.size();

  for(int i = 0 ; i < num_ranges ; i++){
    if(_mask[i] > 0)
      _3d_rays[i] -= _3d_rays[i].dot(temp_n) * temp_n;
  }

  return _3d_rays;
}

const vector<double>& LaserProc::get_ranges() const {
  return _ranges;
}

const vector<double>& LaserProc::get_thetas() const {
  return _thetas;
}

const Eigen::Matrix3d& LaserProc::estimate_fim(double skip_dist, int skip_idxs){
  // ### to be implemented

  _fim = Eigen::Matrix3d::Zero();

  int num_ranges = _ranges.size();

  if(_2d_euclidean_coords_valid == false)
    _polar_to_2d_euclidean();

  // -- Use the rays with mask[i] != 0 to build the environment model.
  //	  These points are called 'valid'.
  // -- Find the orientations of the environment edges.
  // -- Acculumate the information matrix

  vector<int> corner_idxs;
  _alphas.resize(num_ranges);

  for(int c = 1 ; c < _num_clusters ; c++){
    corner_idxs.clear();
    corner_idxs.reserve(num_ranges / 10);

    int prev_valid_idx = -1;
    // Determine the polygon corners by checking 'mask' and the above 
    // two criterion.
    for(int i = 0 ; i < num_ranges ; i++){
      // Is the following data valid?
      if(_mask[i] != c)
        continue;
      // Had we already initialized selecting polygon corners?
      if(prev_valid_idx == -1){
        prev_valid_idx = i;
        corner_idxs.push_back(i);
        // Does the next point comply to become a new polygon corner 
        // (use L1 distance for speed)
      } if(i - prev_valid_idx > skip_idxs ||
          fabs(_2d_rays[i](0) - _2d_rays[prev_valid_idx](0)) +
          fabs(_2d_rays[i](1) - _2d_rays[prev_valid_idx](1)) > skip_dist){
        prev_valid_idx = i;
        corner_idxs.push_back(i);
        //cout << xs[i] << ", " << ys[i] << endl;
      }
    }

    // Add the last point of the '_2d_rays' in order not to miss 
    // possibly significant information.
    for(int i = num_ranges - 1 ; i > corner_idxs.back() ; i--){
      // Is the following data valid?
      if(_mask[i] != c)
        continue;
      corner_idxs.push_back(i);
      break;
    }

    // Determine the orientations of the polygon edges.
    for(int i = 0 ; i < (int)corner_idxs.size() - 1; i++){
      int curr_idx = corner_idxs[i];
      int next_idx = corner_idxs[i+1];
      double dx = _2d_rays[next_idx](0) - _2d_rays[curr_idx](0);
      double dy = _2d_rays[next_idx](1) - _2d_rays[curr_idx](1);
      double alpha = PI/2.0 - atan2(dy, dx);
      //cout << alpha / PI * 180 << endl;
      for(int j = curr_idx ; j <= next_idx ; j++)
        _alphas[j] = alpha;
    }

    // ### This does not take into account the error characteristics of the 
    // laser scanner.
    // Calculate the FIM
    for(int i = 0 ; i < num_ranges ; i++){
      if(_mask[i] != c)
        continue;
      double c = cos(_alphas[i]);
      double s = sin(_alphas[i]);
      double beta = _alphas[i] - (_angle_min + i * _angle_increment);
      double t = tan(beta);
      double z = 1 / cos(beta);
      Eigen::Vector3d u(c * z, s * z, t * _ranges[i]);
      _fim += u * u.transpose();	
      //cout << "u = " << u << endl;
      //cout << "fim = " << fim << endl;
    }
  }
  return _fim;
}

const Eigen::Matrix3d& LaserProc::get_fim() const {
  return _fim;
}

const vector<int>& LaserProc::get_mask() const {
  return _mask;
}

const vector<Eigen::Vector3d>& LaserProc::get_3d_points() const {
  return _3d_rays;
}

const vector<double>& LaserProc::get_linearity_rates() const {
  return _linearity;
}

bool LaserProc::get_RVIZ_markers(visualization_msgs::MarkerArray &marray) const {
  // Get the number of clusters.
  int num_ranges = _ranges.size();

  // The last two markers are for upward/downward looking rays.
  marray.markers.resize(_num_clusters + 2);

  for(int i = 0 ; i < _num_clusters + 2; i++){
    marray.markers[i].points.clear();
    marray.markers[i].colors.clear();
    marray.markers[i].points.reserve(num_ranges/10);
    marray.markers[i].colors.reserve(num_ranges/10);

    marray.markers[i].id = i;
    marray.markers[i].type = visualization_msgs::Marker::SPHERE_LIST;
    marray.markers[i].action = visualization_msgs::Marker::ADD;
    marray.markers[i].pose.position.x = 0;
    marray.markers[i].pose.position.y = 0;
    marray.markers[i].pose.position.z = 0;
    marray.markers[i].pose.orientation.x = 0;
    marray.markers[i].pose.orientation.y = 0;
    marray.markers[i].pose.orientation.z = 0;
    marray.markers[i].pose.orientation.w = 1;
    marray.markers[i].scale.x = 0.05;
    marray.markers[i].scale.y = 0.05;
    marray.markers[i].scale.z = 0.05;
  }

  for(int i = 0 ; i < num_ranges ; i++){
    geometry_msgs::Point pt;
    pt.x = _3d_rays[i](0);
    pt.y = _3d_rays[i](1);
    pt.z = _3d_rays[i](2);
    int idx = _mask[i];
    if(idx < 0)
      idx = _num_clusters + 2 + idx;

    marray.markers[idx].points.push_back(pt);

    marray.markers[idx].color.a = 1.0;
    marray.markers[idx].color.r = _colors[idx](0) / 255.0;
    marray.markers[idx].color.g = _colors[idx](1) / 255.0;
    marray.markers[idx].color.b = _colors[idx](2) / 255.0;
  }

  return true;
}


const vector<pair<int, int> >& LaserProc::get_line_indexes() const {
  return _line_idxs; 
}
