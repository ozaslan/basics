#include "utils.hh"

namespace utils{

Matrix3d yaw2dcm(const double &yaw){
	Matrix3d dcm = Matrix3d::Identity();
	dcm(0, 0) =  cos(yaw);
	dcm(0, 1) =  sin(yaw);
	dcm(1, 0) = -sin(yaw);
	dcm(1, 1) =  cos(yaw);
	return dcm;
}

Matrix3d quat2dcm(const Vector4d &quat){
	double sqw = quat(0) * quat(0);
    double sqx = quat(1) * quat(1);
    double sqy = quat(2) * quat(2);
    double sqz = quat(3) * quat(3);
	
	Matrix3d dcm;

    // invs (inverse square length) is only required if quaternion is not already normalised
    double invs = 1 / (sqx + sqy + sqz + sqw);
    dcm(0, 0) = ( sqx - sqy - sqz + sqw) * invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
    dcm(1, 1) = (-sqx + sqy - sqz + sqw) * invs ;
    dcm(2, 2) = (-sqx - sqy + sqz + sqw) * invs ;
    
    double tmp1 = quat(1) * quat(2);
    double tmp2 = quat(3) * quat(0);
    dcm(1, 0) = 2.0 * (tmp1 + tmp2) * invs ;
    dcm(0, 1) = 2.0 * (tmp1 - tmp2) * invs ;
    
    tmp1 = quat(1) * quat(3);
    tmp2 = quat(2) * quat(0);
    dcm(2, 0) = 2.0 * (tmp1 - tmp2) * invs ;
    dcm(0, 2) = 2.0 * (tmp1 + tmp2) * invs ;

    tmp1 = quat(2) * quat(3);
    tmp2 = quat(1) * quat(0);
    dcm(2, 1) = 2.0 * (tmp1 + tmp2) * invs ;
    dcm(1, 2) = 2.0 * (tmp1 - tmp2) * invs ;      

	return dcm;
}

Vector3d quat2rpy(const Vector4d &quat){
	return dcm2rpy(quat2dcm(quat));
}

Matrix3d rpy2dcm (const Vector3d &rpy){
	double psi = rpy(2); // yaw   - around z
	double phi = rpy(0); // roll  - around interm. x
	double the = rpy(1); // pitch - around interm. y
	
	Matrix3d dcm;

	double cpsi = cos(psi);
	double spsi = sin(psi);
	double cphi = cos(phi);
	double sphi = sin(phi);
	double cthe = cos(the);
	double sthe = sin(the);
	// Due to Mellinger RAM2010 paper.
	// Z-X-Y Euler angles are used to encode the orientation.
	dcm(0, 0) =  cpsi * cthe - sphi * spsi * sthe;
	dcm(0, 1) = -cphi * spsi;
	dcm(0, 2) =  cpsi * sthe + sphi * spsi * cthe;
	dcm(1, 0) =  spsi * cthe + sphi * cpsi * sthe;
	dcm(1, 1) =  cphi * cpsi;
	dcm(1, 2) =  spsi * sthe - sphi * cpsi * cthe;
	dcm(2, 0) = -cphi * sthe;
	dcm(2, 1) =  sphi;
	dcm(2, 2) =  cphi * cthe;

	return dcm;
}

Vector4d rpy2quat(const Vector3d &rpy){
	return dcm2quat(rpy2dcm(rpy));
}

Vector4d dcm2quat(const Matrix3d &dcm){
	double tr = dcm(0, 0) + dcm(1, 1) + dcm(2, 2);
	double qw, qx, qy, qz, S;

	if (tr > 0) { 
		S = sqrt(tr + 1.0) * 2; // S=4*qw 
		qw = 0.25 * S;
		qx = (dcm(2, 1) - dcm(1, 2)) / S;
		qy = (dcm(0, 2) - dcm(2, 0)) / S; 
		qz = (dcm(1, 0) - dcm(0, 1)) / S; 
	} else if ((dcm(0, 0) > dcm(1, 1)) && (dcm(0, 0) > dcm(2, 2))) { 
		S = sqrt(1.0 + dcm(0, 0) - dcm(1, 1) - dcm(2, 2)) * 2; // S=4*qx 
		qw = (dcm(2, 1) - dcm(1, 2)) / S;
		qx = 0.25 * S;
		qy = (dcm(0, 1) + dcm(1, 0)) / S; 
		qz = (dcm(0, 2) + dcm(2, 0)) / S; 
	} else if (dcm(1, 1) > dcm(2, 2)) { 
		S = sqrt(1.0 + dcm(1, 1) - dcm(0, 0) - dcm(2, 2)) * 2; // S=4*qy
		qw = (dcm(0, 2) - dcm(2, 0)) / S;
		qx = (dcm(0, 1) + dcm(1, 0)) / S; 
		qy = 0.25 * S;
		qz = (dcm(1, 2) + dcm(2, 1)) / S; 
	} else { 
		S = sqrt(1.0 + dcm(2, 2) - dcm(0, 0) - dcm(1, 1)) * 2; // S=4*qz
		qw = (dcm(1, 0) - dcm(0, 1)) / S;
		qx = (dcm(0, 2) + dcm(2, 0)) / S;
		qy = (dcm(1, 2) + dcm(2, 1)) / S;
		qz = 0.25 * S;
	}

	return Vector4d(qw, qx, qy, qz);
}

Vector3d dcm2rpy (const Matrix3d &dcm){
	double phi = asin(dcm(2, 1));
	double cosphi = cos(phi);
	double the = atan2(-dcm(2, 0) / cosphi, dcm(2, 2) / cosphi);
	double psi = atan2(-dcm(0, 1) / cosphi, dcm(1, 1) / cosphi);
	return Vector3d(phi, the, psi);
}

Matrix3d cancel_yaw(const Matrix3d &dcm ){
	Vector3d rpy = dcm2rpy(dcm);
	rpy(2) = 0;
	return rpy2dcm(rpy);
}

Vector4d cancel_yaw(const Vector4d &quat){
	Vector3d rpy = quat2rpy(quat);
	rpy(2) = 0;
	return rpy2quat(rpy);
}

// ###
Vector3d slerp(const Vector4d &quat1, const Vector4d &quat2, double theta);

bool get_fim(const sensor_msgs::LaserScan &data, const vector<char> &mask, Eigen::Matrix3d &fim, const char &cluster_id){
	// Either the 'mask' has to be empty which corresponds to all points being valid.
	// Otherwise this function expects the 'data.ranges' and 'mask' to have the same
	// number of elements.
	int mask_size = mask.size();
	int ranges_size = data.ranges.size();
	assert(mask_size == 0 || mask_size == ranges_size);

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


}

