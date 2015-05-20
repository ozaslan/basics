#ifndef __TRANS_UTILS_HH__
#define __TRANS_UTILS_HH__

#include "utils.hh"

namespace utils{
	namespace trans{
		// This function converts nav_msgs::Odomety structure into 
		// its corresponding SE(3) matrix. Hence this only handles
		// rotation and position.
		Matrix4d odom2se3(const nav_msgs::Odometry &odom, bool cancel_yaw = false);
		// This function converts the sensor_msgs::Imu structure into
		// its corresponding SO(3) martix. Hence this only handels
		// orientation.
		Matrix3d imu2dcm(const sensor_msgs::Imu &imu, bool cancel_yaw = false);
		// This function converts geometry_msgs::Pose structure into 
		// its corresponding SE(3) matrix. Hence this only handles
		// rotation and position.
		Matrix4d pose2se3(const geometry_msgs::Pose &pose, bool cancel_yaw = false);
		// The following functions transform between different
		// rotation representations.
		Matrix3d yaw2dcm(const double &yaw);
		Matrix3d quat2dcm(const Vector4d &quat);
		Vector3d quat2rpy(const Vector4d &quat);
		Matrix3d rpy2dcm (const Vector3d &rpy);
		Vector4d rpy2quat(const Vector3d &rpy);
		Vector4d dcm2quat(const Matrix3d &dcm);
		Vector3d dcm2rpy (const Matrix3d &dcm);

		// The following functions cancel the yaw component
		// from the given rotation.
		Matrix3d cancel_yaw(const Matrix3d &dcm );
		Vector4d cancel_yaw(const Vector4d &quat);

		// ### This function interpolates quaternions using the 
		// SLERP algorithm.
		Vector3d slerp(const Vector3d &quat1, const Vector3d &quat2, double theta);
	}
}

#endif
