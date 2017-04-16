#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

// This step should be the same for both the sources
void KalmanFilter::Predict()
{
  // State
  x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();

  // Covariance
	P_ = F_ * P_ * Ft + Q_;
}

// This applies for Laser/Lidar
void KalmanFilter::Update(const VectorXd &z)
{
  // Update the state by using Kalman Filter equations

  VectorXd y = z - H_laser_ * x_;
	MatrixXd Ht = H_laser_.transpose();
	MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// New estimate
	x_ = x_ + (K * y);

  // Create the identity matrix of the same size
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

  // New covariance
	P_ = (I - K * H_laser_) * P_;
}

// This applies for Radar
void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  // Update the state by using Extended Kalman Filter equations
  // h(x') to map from cartesian to polar co-ordinates
  VectorXd h_x = CartesianToPolar(x_);

  // Calculate the error
  VectorXd y = z - h_x;

  // Calculate the jacobian measurement matrix
  Hj_radar_ = CalculateJacobian(x_);

  // The remaining questions are the same
	MatrixXd Hjt = Hj_radar_.transpose();
	MatrixXd S = Hj_radar_ * P_ * Hjt + R_radar_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Hjt;
	MatrixXd K = PHt * Si;

	// New estimate
	x_ = x_ + (K * y);

  // Create the identity matrix of the same size
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

  // New covariance
	P_ = (I - K * Hj_radar_) * P_;
}

// Function to calculate h(x') for cartesian to polar conversion
VectorXd KalmanFilter::CartesianToPolar(const VectorXd &x_state)
{
  // h_x is a 3 element vector
  VectorXd h_x = VectorXd(3);

  // Recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Create some variables for easier computation
  float px_2_py_2 = pow(px, 2) + pow(py, 2);

  // Check division by zero
  if(fabs(px_2_py_2) < 0.0001)
  {
    // Set to a very low value to avoid divide by 0 error
    px_2_py_2 = 0.0001;
  }

  float sqrt_px_py = sqrt(px_2_py_2);

  // h(x')
  h_x << sqrt_px_py, atan2(py, px), (px*vx + py*vy)/sqrt_px_py;

  return h_x;
}

// Function to calculate the jacobian for the predicted state passed
MatrixXd KalmanFilter::CalculateJacobian(const VectorXd &x_state)
{
  // Hj is a 3x4 matrix
  MatrixXd Hj(3,4);

  // Recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Create some variables for easier computation
  float px_2_py_2 = pow(px, 2) + pow(py, 2);

  // Check division by zero
  if(fabs(px_2_py_2) < 0.0001)
  {
    // Return with the initial values set to 0
    // If H=0, then Kalman gain 'K' = 0. So x=x'.
    // The state vector won't get updated with the new
    // measurement update.
    Hj << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

    return Hj;
  }

  float sqrt_px_py = sqrt(px_2_py_2);
  float vxpy_min_vypx = vx*py - vy*px;
  float vypx_min_vxpy = vy*px - vx*py;

  // Compute the Jacobian matrix
  Hj << px/sqrt_px_py, py/sqrt_px_py, 0, 0,
        -py/px_2_py_2, px/px_2_py_2, 0, 0,
        py*vxpy_min_vypx/pow(px_2_py_2, 1.5), px*vypx_min_vxpy/pow(px_2_py_2, 1.5), px/sqrt_px_py, py/sqrt_px_py;

  return Hj;
}
