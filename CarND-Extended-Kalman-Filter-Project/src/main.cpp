#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {

  check_arguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  string line;

  // Prep the measurement packages (each line represents a measurement at a
  // particular timestamp)
  while (getline(in_file_, line))
  {
    // String object for storing the sensor type
    string sensor_type;
    // Measurement package object
    MeasurementPackage meas_package;
    // Ground truth package(actual values)
    GroundTruthPackage gt_package;
    // Object which breaks down the passed in string into words
    istringstream iss(line);
    // Variable for storing the time stamp
    long long timestamp;

    // Get the first word in the line(or character)
    iss >> sensor_type;

    // LASER/LIDAR MEASUREMENT
    if (sensor_type.compare("L") == 0)
    {
      // Store the measurement source
      meas_package.sensor_type_ = MeasurementPackage::LASER;

      // Vector for storing the position
      meas_package.raw_measurements_ = VectorXd(2);

      // Variables to store x and y values(px, py)
      float x;
      float y;

      // Get the next two words which are measurements
      iss >> x;
      iss >> y;

      // Store the position values
      meas_package.raw_measurements_ << x, y;

      // Get the time stamp
      iss >> timestamp;

      // Store the timestamp
      meas_package.timestamp_ = timestamp;

      // Push this measurement into the vector
      measurement_pack_list.push_back(meas_package);
    }
    // RADAR MEASUREMENT, don't do anything for now
    else if (sensor_type.compare("R") == 0)
    {
      // Store the measurement source
      meas_package.sensor_type_ = MeasurementPackage::RADAR;

      // Vector for storing the position
      meas_package.raw_measurements_ = VectorXd(3);

      // Variables to store the rho(range), phi(angle) and rho_dot(magnitude)
      float ro;
      float phi;
      float ro_dot;

      // Get the respective values
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;

      // Store the measurements in the vector
      meas_package.raw_measurements_ << ro, phi, ro_dot;

      // Get the timestamp
      iss >> timestamp;

      // Store the timestamp
      meas_package.timestamp_ = timestamp;

      // Push the values into the vector
      measurement_pack_list.push_back(meas_package);
    }

    // Read ground truth data to compare later(error/accuracy calculation)
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;

    // Get the ground truth values
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    // An empty vector
    gt_package.gt_values_ = VectorXd(4);

    // Push the ground truth
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;

    // Add the values to the vector
    gt_pack_list.push_back(gt_package);
  }

  // Create a Fusion EKF instance
  FusionEKF fusionEKF;

  // Used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // Call the EKF-based fusion
  size_t N = measurement_pack_list.size();

  // For each of the measurements in the vector
  for (size_t k = 0; k < N; k++)
  {
    // Start filtering from the second frame, so the initial state needs to be
    // set in the EKF object routines as the speed is unknown in the first frame
    fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

    // Output the estimation
    out_file_ << fusionEKF.ekf_.x_(0) << "\t";
    out_file_ << fusionEKF.ekf_.x_(1) << "\t";
    out_file_ << fusionEKF.ekf_.x_(2) << "\t";
    out_file_ << fusionEKF.ekf_.x_(3) << "\t";

    // Output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER)
    {
      // Output the estimation
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    }
    else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR)
    {
      // Output the estimation in the cartesian coordinates
      float ro = measurement_pack_list[k].raw_measurements_(0);
      float phi = measurement_pack_list[k].raw_measurements_(1);

      out_file_ << ro * cos(phi) << "\t"; // p1_meas
      out_file_ << ro * sin(phi) << "\t"; // ps_meas
    }

    // Output the ground truth vaues
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

    // Push the estimations and the ground truth
    estimations.push_back(fusionEKF.ekf_.x_);
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }

  // Compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;

  // Close all the files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}
