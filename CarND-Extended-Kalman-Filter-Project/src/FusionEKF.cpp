#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Constructor
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // Initializing measurement noise covariance matrix for Laser
  R_laser_ = MatrixXd(2, 2);

  // For Radar
  R_radar_ = MatrixXd(3, 3);

  // Measurement matrix for Laser
  H_laser_ = MatrixXd(2, 4);

  // Jacobian Measurement matrix radar
  Hj_ = MatrixXd(3, 4);

  // Acceleration noise components(sigma) for calculation of
  // the process noise covariance matrix Q
  noise_ax = 9;
  noise_ay = 9;

  // Measurement noise covariance matrix, laser, values
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // Measurement noise covariance matrix, radar, values
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
}

// Destructor
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  // Check if the initialization is done i.e. this is the first measruement
  if (!is_initialized_) {
    // Initialize the state ekf_.x_ with the first measurement.
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    // Done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  // Prediction step
  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  // Update step

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

  // Print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
