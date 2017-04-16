#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

# define DEBUG_EKF_OUTPUT 0

class FusionEKF
{
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

  // Acceleration noise components(sigma)
  float noise_ax;
  float noise_ay;
};

#endif /* FusionEKF_H_ */
