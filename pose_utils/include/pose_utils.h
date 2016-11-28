#ifndef POSE_UTILS_H
#define POSE_UTILS_H

#include <iostream>
#include "armadillo"

#define PI 3.14159265359
#define NUM_INF 999999.9

using namespace std;

// Rotation ---------------------
arma::mat ypr_to_R(const arma::colvec& ypr);

arma::mat yaw_to_R(double yaw);

arma::colvec R_to_ypr(const arma::mat& R);

arma::mat quaternion_to_R(const arma::colvec& q);

arma::colvec R_to_quaternion(const arma::mat& R);

arma::colvec quaternion_mul(const arma::colvec& q1, const arma::colvec& q2);

arma::colvec quaternion_inv(const arma::colvec& q);

// General Pose Update ----------
arma::colvec pose_update(const arma::colvec& X1, const arma::colvec& X2);

arma::colvec pose_inverse(const arma::colvec& X);

arma::colvec pose_update_2d(const arma::colvec& X1, const arma::colvec& X2);

arma::colvec pose_inverse_2d(const arma::colvec& X);

// For Pose EKF -----------------
arma::mat Jplus1(const arma::colvec& X1, const arma::colvec& X2);

arma::mat Jplus2(const arma::colvec& X1, const arma::colvec& X2);

// For IMU EKF ------------------
arma::colvec state_update(const arma::colvec& X, const arma::colvec& U, double dt);

arma::mat jacobianF(const arma::colvec& X, const arma::colvec& U, double dt);

arma::mat jacobianU(const arma::colvec& X, const arma::colvec& U, double dt);

arma::colvec state_measure(const arma::colvec& X);

arma::mat jacobianH();

#endif
