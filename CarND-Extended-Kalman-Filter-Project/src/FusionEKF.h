#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  // Constructor
  FusionEKF();

  // Destructor
  virtual ~FusionEKF();

  // Runs the whole flow of the Kalman Filter
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  // Kalman Filter update and prediction math lives in object
  KalmanFilter ekf_;

private:
  // Check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // Previous timestamp
  long previous_timestamp_;

  // Tool object used to compute Jacobian and RMSE
  Tools tools;

  // Measurement noise covariance matrix for laser/lidar
  Eigen::MatrixXd R_laser_;

  // Measurement noise covariance matrix for radar
  Eigen::MatrixXd R_radar_;

  // H measurement matrix for laser/lidar
  Eigen::MatrixXd H_laser_;

  // H jacobian measurement matrix for radar(for non-linear to linear conversion)
  Eigen::MatrixXd Hj_;
};

#endif /* FusionEKF_H_ */
