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

  // Create a 4D state vector
  ekf_.x_ = VectorXd(4);

  // State covariance matrix 'P'
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  // The initial transition matrix 'F'
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // The process noise covariance matrix 'Q'
  ekf_.Q_ = MatrixXd(4, 4);

  // Measurement matrix for Laser
  ekf_.H_laser_ = MatrixXd(2, 4);
  ekf_.H_laser_ << 1, 0, 0, 0,
                   0, 1, 0, 0;

  // Jacobian Measurement matrix for Radar
  // NOTE: This matrix doesn't need to be initialized because
	//       h(x') function is used in the error calculation step
	//       i.e. calculation of y = z - h(x').
  //       The Jacobian Matrix is calculated on the fly
  ekf_.Hj_radar_ = MatrixXd(3, 4);

  // Initializing measurement noise covariance matrix for Laser
  ekf_.R_laser_ = MatrixXd(2, 2);
  ekf_.R_laser_ << 0.0225, 0,
                   0, 0.0225;

  // Initializing measurement noise covariance matrix for Radar
  ekf_.R_radar_ = MatrixXd(3, 3);
  ekf_.R_radar_ << 0.09, 0, 0,
                   0, 0.0009, 0,
                   0, 0, 0.09;

  // Acceleration noise components(sigma) for calculation of
  // the process noise covariance matrix Q
  noise_ax = 9;
  noise_ay = 9;

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
    ekf_.x_ = VectorXd(4);

    // Lidar measurement
    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      // Set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }
    // Radar measurement
    else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      // Get the measurements
      float r = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];

      // Convert radar from polar to cartesian coordinates and initialize state.
      float px = r * cos(theta);
      float py = r * sin(theta);

      // The initial velocity is zero as the sensors don't measure velocity
      ekf_.x_ << px, py, 0, 0;
    }

    // Capture the timestamp for the next iteration
		previous_timestamp_ = measurement_pack.timestamp_;

    // Done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // Compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;

  // Capture the timestamp for the next iteration
  previous_timestamp_ = measurement_pack.timestamp_;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Create the process noise covariance matrix.
  float dt_2 = pow(dt, 2);
  float dt_3 = pow(dt, 3);
  float dt_4 = pow(dt, 4);

  ekf_.Q_ << dt_4/4 * noise_ax, 0, dt_3/2 * noise_ax, 0,
             0, dt_4/4 * noise_ay, 0, dt_3/2 * noise_ay,
             dt_3/2 * noise_ax, 0, dt_2 * noise_ax, 0,
             0, dt_3/2 * noise_ay, 0, dt_2 * noise_ay;

  // Prediction step
  ekf_.Predict();

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  // Update step
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  } else {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }

  // Print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
