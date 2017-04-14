#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

// This step should be the same for both the sources
void KalmanFilter::Predict() {
  // Predict the state
  // Position
  //x_ = F_ * x_;
	//MatrixXd Ft = F_.transpose();
  // Covariance
	//P_ = F_ * P_ * Ft + Q_;
}

// This applies for Laser/Lidar
void KalmanFilter::Update(const VectorXd &z) {
  // Update the state by using Kalman Filter equations

  /*
  VectorXd y = z - H_ * x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// New estimate
	x_ = x_ + (K * y);

  // Create the identity matrix of the same size
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // New covariance
	P_ = (I - K * H_) * P_;
  */
}

// This applies for Radar
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Update the state by using Extended Kalman Filter equations

  /*
  // Recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Create some variables for easier computation
   float px_2_py_2 = pow(px, 2) + pow(py, 2);
   float sqrt_px_py = sqrt(px_2_py_2);
   float vxpy_min_vypx = vx*py - vy*px;
   float vypx_min_vxpy = vy*px - vx*py;

   // Check division by zero
   if(fabs(px_2_py_2) < 0.0001)
   {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

   // Compute the Jacobian matrix
   Hj << px/sqrt_px_py, py/sqrt_px_py, 0, 0,
         -py/px_2_py_2, px/px_2_py_2, 0, 0,
         py*vxpy_min_vypx/pow(px_2_py_2, 1.5), px*vypx_min_vxpy/pow(px_2_py_2, 1.5), px/sqrt_px_py, py/sqrt_px_py;
  */
}
