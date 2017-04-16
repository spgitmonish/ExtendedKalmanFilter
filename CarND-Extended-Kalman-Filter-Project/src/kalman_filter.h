#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include <math.h>

class KalmanFilter
{
public:
  // State vector
  Eigen::VectorXd x_;

  // State covariance matrix
  Eigen::MatrixXd P_;

  // State transistion matrix
  Eigen::MatrixXd F_;

  // Process noise covariance matrix
  Eigen::MatrixXd Q_;

  // Measurement matrix for Laser
  Eigen::MatrixXd H_laser_;

  // Measurement Jacobian matrix for Radar
  Eigen::MatrixXd Hj_radar_;

  // Measurement noise covariance matrix for Laser
  Eigen::MatrixXd R_laser_;

  // Measurement noise covariance matrix for Radar
  Eigen::MatrixXd R_radar_;

  // Constructor
  KalmanFilter();

  // Destructor
  virtual ~KalmanFilter();

  // Predict Predicts the state and the state covariance
	// using the process model
	void Predict();

	// Updates the state by using Kalman Filter and
  // the measurement(z) at k+1
  void Update(const Eigen::VectorXd &z);

  // Updates the state by using Extended Kalman Filter and
  // the measurement(z) at k+1
  void UpdateEKF(const Eigen::VectorXd &z);

private:
  // Private Function to calculate the Jacobian matrix for the
  // predicted state
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);

  // Private function which converts the cartesian coordinates to
  // polar coordinates
  Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd &x_state);
};

#endif /* KALMAN_FILTER_H_ */
