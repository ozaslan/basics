#include "trans_utils.hh"

namespace utils{
	namespace trans{
		Matrix4d pose2se3(const geometry_msgs::Pose &pose, bool cncl_yaw){
			Matrix4d trans;
			trans.fill(0);
			trans(0, 3) = pose.position.x;
			trans(1, 3) = pose.position.y;
			trans(2, 3) = pose.position.z;
			trans(3, 3) = 1;
			Vector4d quat;
			quat(0) = pose.orientation.w;
			quat(1) = pose.orientation.x;
			quat(2) = pose.orientation.y;
			quat(3) = pose.orientation.z;
			if(cncl_yaw == true){
				Matrix3d dcm = quat2dcm(quat); 
				trans.topLeftCorner(3, 3) = cancel_yaw(dcm); 
			} else
				trans.topLeftCorner<3, 3>() = quat2dcm(quat);
			return trans;
		}

		Matrix4d odom2se3(const nav_msgs::Odometry &odom, bool cncl_yaw){
			Matrix4d pose;
			pose.fill(0);
			pose(0, 3) = odom.pose.pose.position.x;
			pose(1, 3) = odom.pose.pose.position.y;
			pose(2, 3) = odom.pose.pose.position.z;
			pose(3, 3) = 1;
			Vector4d quat;
			quat(0) = odom.pose.pose.orientation.w;
			quat(1) = odom.pose.pose.orientation.x;
			quat(2) = odom.pose.pose.orientation.y;
			quat(3) = odom.pose.pose.orientation.z;
			if(cncl_yaw == true){
				Matrix3d dcm = quat2dcm(quat); 
				pose.topLeftCorner(3, 3) = cancel_yaw(dcm); 
			} else
				pose.topLeftCorner<3, 3>() = quat2dcm(quat);
			return pose;
		}

		Matrix3d imu2dcm(const sensor_msgs::Imu &imu, bool cncl_yaw){
			Vector4d quat;
			Matrix3d dcm;
			quat(0) = imu.orientation.w;
			quat(1) = imu.orientation.x;
			quat(2) = imu.orientation.y;
			quat(3) = imu.orientation.z;
			dcm = quat2dcm(quat); 
			if(cncl_yaw == true)
				dcm = cancel_yaw(dcm); 
			return dcm;
		}


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

	}
}

