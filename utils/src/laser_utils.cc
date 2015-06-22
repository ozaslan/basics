#include "trans_utils.hh"

namespace utils{
  namespace laser{

    bool get_fim(const sensor_msgs::LaserScan &data, const vector<char> &mask, Eigen::Matrix3d &fim, const char &cluster_id){
      // Either the 'mask' has to be empty which corresponds to all points being valid.
      // Otherwise this function expects the 'data.ranges' and 'mask' to have the same
      // number of elements.
      int mask_size = mask.size();
      int ranges_size = data.ranges.size();
      ASSERT(mask_size == 0 || mask_size == ranges_size, "mask_size == 0 || mask_size == data.ranges.size()");

      //cout << "-- M1" << endl;

      // Either of the followings should hold before adding a new polygon corner
      static double skip_dist = 0.3;	// Minimum distance between consecutive polygon corners.
      static int	  skip_idxs = 30;	// Minimum indicial distance between ranges data used to
      // represent environment polygon.

      //cout << "-- M2" << endl;
      static vector<double> xs, ys, ths;
      xs.resize(ranges_size);
      ys.resize(ranges_size);
      ths.resize(ranges_size);
      for(int i = 0 ; i < ranges_size ; i++){
        ths[i] = data.angle_min + i * data.angle_increment;
        if(mask_size == 0 || mask[i] == cluster_id )
          polar2euclidean(data.ranges[i], ths[i], xs[i], ys[i]);
      }

      //cout << "-- M3" << endl;
      // -- Use the rays with mask[i] != 0 to build the environment model.
      //	  These points are called 'valid'.
      // -- Find the orientations of the environment edges.
      // -- Acculumate the information matrix
      static vector<double> alphas;
      vector<int> corner_idxs;
      alphas.resize(ranges_size);
      corner_idxs.reserve(ranges_size / 10);

      //cout << "-- M4" << endl;
      int prev_valid_idx = -1;
      // Determine the polygon corners by checking 'mask' and the above 
      // two criterion.
      for(int i = 0 ; i < ranges_size ; i++){
        // Is the following data valid?
        if(mask_size != 0 && mask[i] != cluster_id)
          continue;
        // Had we already initialized selecting polygon corners?
        if(prev_valid_idx == -1){
          prev_valid_idx = i;
          corner_idxs.push_back(i);
          // Does the next point comply to become a new polygon corner 
          // (use L1 distance for speed)
        } if(i - prev_valid_idx > skip_idxs ||
            fabs(xs[i] - xs[prev_valid_idx]) +
            fabs(ys[i] - ys[prev_valid_idx]) > skip_dist){
          prev_valid_idx = i;
          corner_idxs.push_back(i);
          //cout << xs[i] << ", " << ys[i] << endl;
        }
      }
      //cout << "-- M5" << endl;
      // Add the last point of the 'data.ranges' in order not to miss 
      // possibly significant information.
      for(int i = ranges_size - 1 ; i > corner_idxs.back() ; i--){
        // Is the following data valid?
        if(mask_size != 0 && mask[i] != cluster_id)
          continue;
        corner_idxs.push_back(i);
        break;
      }
      //cout << "# of polygon corners : " << corner_idxs.size() << endl;
      //cout << "-- M6" << endl;
      // Determine the orientations of the polygon edges.
      for(int i = 0 ; i < (int)corner_idxs.size() - 1; i++){
        int curr_idx = corner_idxs[i];
        int next_idx = corner_idxs[i+1];
        double dx = xs[next_idx] - xs[curr_idx];
        double dy = ys[next_idx] - ys[curr_idx];
        double alpha = PI/2.0 - atan2(dy, dx);
        //cout << alpha / PI * 180 << endl;
        for(int j = curr_idx ; j <= next_idx ; j++)
          alphas[j] = alpha;
      }

      //cout << "-- M7" << endl;
      // ### This does not take into account the error characteristics of the 
      // laser scanner.
      // Calculate the FIM
      fim = Eigen::Matrix3d::Zero();
      for(int i = 0 ; i < ranges_size ; i++){
        if(mask_size != 0 && mask[i] != cluster_id)
          continue;
        double c = cos(alphas[i]);
        double s = sin(alphas[i]);
        double beta = alphas[i] - ths[i];
        double t = tan(beta);
        double z = 1 / cos(beta);
        Eigen::Vector3d u(c * z, s * z, t * data.ranges[i]);
        fim += u * u.transpose();	
        //cout << "u = " << u << endl;
        //cout << "fim = " << fim << endl;
      }

      //cout << "-- M8" << endl;
      return true;
    }

    bool get_fim(const sensor_msgs::PointCloud2 &data, const vector<char> &mask, Eigen::Matrix6d &fim){
      return true;
    }

    bool cluster_laser_scan(const sensor_msgs::LaserScan &data, vector<char> &mask, int &num_clusters,
        double min_range, double max_range, int min_skips,
        double range_jump, int min_cluster_size){
      bool has_occlusion = false;
      if(mask.size() == data.ranges.size() && num_clusters == 1)
        has_occlusion = true;
      int num_ranges = data.ranges.size();
      mask.resize(num_ranges);
      char current_label = 1;
      int current_cluster_size = 0;
      int skipped_rays = 0;
      double prev_range, range;
      prev_range = -999;

      /*
         cout << endl;
         cout << "mask input" << endl;
         for(int i = 0 ; i < mask.size() ; i++)
         cout << (int)mask[i]; 
         cout << endl;
       */

      for(int i = 0 ; i < num_ranges ; i++){
        // Check if there is occlusion (angle spans required to
        // be omitted)
        if(has_occlusion == true && mask[i] == 0)
          continue;
        range = data.ranges[i];
        // Check if the 'range' is a proper value
        if(range != range || range < min_range || 
            range > max_range || !isfinite(range)){
          mask[i] = 0;
          skipped_rays++;
          continue;
        }
        // Should a new cluster be initialized?
        // If positive, then also check the size of the previous cluster
        // and delete if the number of rays in it is smaller than the
        // minimum allowable number of rays
        if(skipped_rays >= min_skips || fabs(range - prev_range) > range_jump){
          // Before initializing a new cluster, check 
          // the size of the previous one. If the size is
          // small, delete it.
          if(current_cluster_size < min_cluster_size){
            for(int j = i - 1 ; j >= 0 ; j--)
              if(mask[j] == current_label)
                mask[j] = 0;
              else if(mask[j] == current_label - 1)
                break;
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
        mask[i] = current_label;
      }

      // Before returning check again the total number of
      // rays in the last cluster. Delete if it is too small.
      if(current_cluster_size < min_cluster_size){
        for(int j = num_ranges - 1 ; j >= 0 ; j--)
          if(mask[j] == current_label)
            mask[j] = 0;
          else if(mask[j] == current_label - 1)
            break;
        current_label--;
      }

      num_clusters = current_label;

      /*
         cout << endl;
         cout << "mask output" << endl;
         for(int i = 0 ; i < mask.size() ; i++)
         cout << (int)mask[i];
         cout << endl;
       */
      return true;
    }


    bool fit_lines(const sensor_msgs::LaserScan &data, const vector<char> &mask, vector<pair<int, int> > &line_idxs,
        double min_len, double min_pts, double epsilon){
      int mask_size = mask.size();
      int ranges_size = data.ranges.size();
      ASSERT(mask_size == 0 || mask_size == ranges_size, "mask.size() == 0 || mask.size() == ranges.size())");

      vector<cv::Point2f> raw_pts, poly_pts;
      raw_pts.reserve(ranges_size);
      for(int i = 0 ; i < ranges_size ; i++){
        double th = data.angle_min + i * data.angle_increment;
        double x, y;
        if(mask_size == 0 || mask[i] != false){
          polar2euclidean(data.ranges[i], th, x, y);
          raw_pts.push_back(cv::Point2d(x, y));
        }
      }

      //cout << "raw_pts.size() == " << raw_pts.size() << endl;
      if(raw_pts.size() == 0){
        line_idxs.clear();
        return false;
      }

      cv::approxPolyDP(raw_pts, poly_pts, epsilon, false);

      //cout << "# of poly_pts = " << poly_pts.size() << endl;

      vector<int> corner_idxs;
      corner_idxs.reserve(poly_pts.size());

      for(int i = 0, r = 0, p = 0 ; i < (int)ranges_size && 
          p < (int)poly_pts.size() && 
          r < (int)raw_pts.size() ; i++){
        if(mask_size == 0 || mask[i] != false){
          if(raw_pts[r++] == poly_pts[p]){
            corner_idxs.push_back(i);
            p++;
          }
        }
      }

      //cout << "corner_idxs.size() = " << corner_idxs.size() << endl;

      for(int i = 0 ; i < (int)corner_idxs.size() - 1; i++){
        //cout << "[" << i << "] = " << corner_idxs[i] << endl;
        int start_idx = corner_idxs[i];
        int end_idx   = corner_idxs[i+1];
        if(mask[start_idx + 1] != false){
          if(end_idx - start_idx + 1 >= min_pts){
            cv::Point2d diff = poly_pts[i+1] - poly_pts[i];
            if(diff.dot(diff) >= min_len * min_len)
              line_idxs.push_back(make_pair<int, int>(start_idx, end_idx));
          }
        }
      }

      return true;
    }

    Eigen::Vector5d register_scan(const vector<double> &ranges1, const vector<int> &mask1, const vector<double> &ths1,
        const vector<double> &ranges2, const vector<int> &mask2, const vector<double> &ths2,
        const Eigen::Vector3d &init_pose,
        bool  recover_from_error, double max_angular_correction_deg, double max_linear_correction,
        int   max_iterations,	double epsilon_xy, double epsilon_theta, double max_correspondence_dist, 
        double sigma, bool use_corr_tricks,	bool restart, double restart_threshold_mean_error, double restart_dt,
        double restart_dtheta){

      struct sm_params params;
      struct sm_result result;

      params.max_angular_correction_deg = max_angular_correction_deg;
      params.max_linear_correction = max_linear_correction;
      params.max_iterations = max_iterations;
      params.epsilon_xy = epsilon_xy;
      params.epsilon_theta = epsilon_theta;
      params.max_correspondence_dist = max_correspondence_dist;
      params.sigma = sigma;
      params.restart = restart == true ? 1 : 0;
      params.restart_threshold_mean_error = restart_threshold_mean_error;
      params.restart_dt = restart_dt;
      params.restart_dtheta = restart_dtheta;

      params.use_ml_weights = 0;
      params.debug_verify_tricks = 0;

      params.clustering_threshold = 0.05;
      params.orientation_neighbourhood = 3;
      params.use_corr_tricks = 1;

      params.do_alpha_test = 0;
      params.outliers_maxPerc = 0.85;

      params.outliers_adaptive_order = 0.7;
      params.outliers_adaptive_mult = 2;
      params.do_visibility_test = 0;
      params.do_compute_covariance = 0;
      params.use_sigma_weights = 0;

      struct laser_data ld1, ld2;

      int num_valid_ranges1, num_valid_ranges2;
      double min_angle1, min_angle2;
      double max_angle1, max_angle2;
      min_angle1 =  99999;
      min_angle2 =  99999;
      max_angle1 = -99999;
      max_angle2 = -99999;
      num_valid_ranges1 = 0;
      num_valid_ranges2 = 0;

      for(int i = 0 ; i < (int)mask1.size() ; i++){
        if(mask1[i] > 0){
          num_valid_ranges1++;
          if(ths1[i] < min_angle1)
            min_angle1 = ths1[i];
          if(ths1[i] > max_angle1)
            max_angle1 = ths1[i];
        }
      }

      for(int i = 0 ; i < (int)mask2.size() ; i++){
        if(mask2[i] > 0){
          num_valid_ranges2++;
          if(ths2[i] < min_angle2)
            min_angle2 = ths2[i];
          if(ths2[i] > max_angle2)
            max_angle2 = ths2[i];
        }
      }

      //cout << "num_valid_ranges1 = " << num_valid_ranges1 << endl;
      //cout << "num_valid_ranges2 = " << num_valid_ranges2 << endl;


      ld_alloc(&ld1, num_valid_ranges1);
      ld_alloc(&ld2, num_valid_ranges2);

      /*
        ld_alloc(&ld1, mask1.size());
      ld_alloc(&ld2, mask2.size());
      */
  
      ld1.nrays = num_valid_ranges1;
      ld1.min_theta = min_angle1;
      ld1.max_theta = max_angle1;
      for(int i = 0, j = 0 ; i < (int)mask1.size() ; i++){
        if(mask1[i] > 0){
          ld1.theta[j] = ths1[i];
          ld1.valid[j] = 1;
          ld1.readings[j] = ranges1[i];
          j++;
        }
      }

      /*
      ld1.nrays = mask1.size();
      ld1.min_theta = *std::min_element(ths1.begin(), ths1.end());
      ld1.max_theta = *std::max_element(ths1.begin(), ths1.end());
      memcpy(ld1.theta   , &ths1[0]     , mask1.size() * sizeof(ths1[0]));
      memcpy(ld1.valid   , &mask1[0]    , mask1.size() * sizeof(mask1[0]));
      memcpy(ld1.readings, &ranges1[0]  , mask1.size() * sizeof(ranges1[0]));
      */

      //cout << "ld1_min_theta = " << ld1.min_theta << endl;
      //cout << "ld1.theta[0]  = " << ld1.theta[0]  << endl;

      ld2.nrays = num_valid_ranges2;
      ld2.min_theta = min_angle2;
      ld2.max_theta = max_angle2;
      for(int i = 0, j = 0 ; i < (int)mask2.size() ; i++){
        if(mask2[i] > 0){
          ld2.theta[j] = ths2[i];
          ld2.valid[j] = 1;
          ld2.readings[j] = ranges2[i];
          j++;
        }
      }

      /*
      ld2.nrays = mask2.size();
      ld2.min_theta = *std::min_element(ths2.begin(), ths2.end());
      ld2.max_theta = *std::max_element(ths2.begin(), ths2.end());
      memcpy(ld2.theta   , &ths2[0]     , mask2.size() * sizeof(ths2[0]));
      memcpy(ld2.valid   , &mask2[0]    , mask2.size() * sizeof(mask2[0]));
      memcpy(ld2.readings, &ranges2[0]  , mask2.size() * sizeof(ranges2[0]));
      */

      //cout << "ld2_min_theta = " << ld2.min_theta << endl;
      //cout << "ld2.theta[0]  = " << ld2.theta[0]  << endl;

      /*
      for(int i = 0 ; i < (int)mask1.size() ; i++){
        ld1.valid[i]  = mask1[i] <= 0 ? 0 : 1;
        if(mask1[i] <= 0)
          ld1.readings[i] = numeric_limits<double>::quiet_NaN();
      }

      for(int i = 0 ; i < (int)mask2.size() ; i++){
        ld2.valid[i]  = mask2[i] <= 0 ? 0 : 1;
        if(mask2[i] <= 0)
          ld2.readings[i] = numeric_limits<double>::quiet_NaN();
      }
      */

      ld1.odometry[0] = 0;
      ld1.odometry[1] = 0;
      ld1.odometry[2] = 0;
      ld2.odometry[0] = 0;
      ld2.odometry[1] = 0;
      ld2.odometry[2] = 0;

      params.laser_ref  = &ld1;
      params.laser_sens = &ld2;
      params.first_guess[0] = init_pose(0);
      params.first_guess[1] = init_pose(1);
      params.first_guess[2] = init_pose(2);

      params.min_reading = 0;
      params.max_reading = 20;

      /*
         cout << params.laser_ref->nrays << ", " << params.laser_ref->min_theta << ", " << params.laser_ref->max_theta << endl;
         for(int i = 0 ; i < ld1.nrays ; i++)
         cout << params.laser_ref->theta[i] << " ";
         cout << endl;
         for(int i = 0 ; i < ld1.nrays ; i++)
         cout << params.laser_ref->valid[i] << " ";
         cout << endl;
         for(int i = 0 ; i < ld1.nrays ; i++)
         cout << params.laser_ref->readings[i] << " ";
         cout << endl;
         cout << "----------------------" << endl;
         cout << params.laser_sens->nrays << ", " << params.laser_sens->min_theta << ", " << params.laser_sens->max_theta << endl;
         for(int i = 0 ; i < ld2.nrays ; i++)
         cout << params.laser_sens->theta[i] << " ";
         cout << endl;
         for(int i = 0 ; i < ld2.nrays ; i++)
         cout << params.laser_sens->valid[i] << " ";
         cout << endl;
         for(int i = 0 ; i < ld2.nrays ; i++)
         cout << params.laser_sens->readings[i] << " ";
         cout << endl;

       */
      sm_icp(&params, &result);

      ld_dealloc(&ld1);
      ld_dealloc(&ld2);

      if(!result.valid){
        return Eigen::Vector5d::Zero();
      } else {
        Eigen::Vector5d trans;
        trans(0) = result.x[0];
        trans(1) = result.x[1];
        trans(2) = result.x[2];
        trans(3) = result.nvalid;
        trans(4) = result.error;
        return trans;
      }

      //cout << result.valid << ", " << result.iterations << ", [" << result.x[0] << ", " << result.x[1] << ", " << result.x[2] << "]" << endl;
      //cout << result.nvalid << ", " << result.error << endl;       
    }
  }
}






