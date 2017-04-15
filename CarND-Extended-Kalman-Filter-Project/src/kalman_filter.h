#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include <math.h>

class KalmanFilter {
public:

  // State vector
  Eigen::VectorXd x_;

  // State covariance matrix
  Eigen::MatrixXd P_;

  // State transistion matrix
  Eigen::MatrixXd F_;

  // Process noise covariance matrix
  Eigen::MatrixXd Q_;

  // Measurement matrix
  Eigen::MatrixXd H_;

  // Measurement noise covariance matrix
  Eigen::MatrixXd R_;

  // Constructor
  KalmanFilter();

  // Destructor
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
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
