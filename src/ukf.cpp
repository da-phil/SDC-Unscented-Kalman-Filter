#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  timestep_ = 0;
  is_initialized_ = false;
  ///* State dimension
  n_x_ = 5;
  ///* Augmented state dimension
  n_aug_ = 7;
  ///* Sigma point spreading parameter
  lambda_ = 3;
  ///* Weights of sigma points
  weights_ = VectorXd::Zero(2*n_aug_+1);
  time_us_ = 0;
  x_ = VectorXd::Zero(n_x_);
  P_ = MatrixXd::Identity(n_x_, n_x_);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  float dt;
 
  //compute the time elapsed between the current and previous measurements
  dt = (meas_package.timestamp_ - time_us_) / 1.0e6; //time in seconds
  time_us_ = meas_package.timestamp_;

  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_) {
    if (!is_initialized_) {
        /**
        Initialize state by lidar measurement.
        */
        cout << "Initial lidar measurement received!" << endl;
        float px, py, v, yaw, yawd;
        px = meas_package.raw_measurements_[0];
        py = meas_package.raw_measurements_[1];
        v  = 0.0;
        yaw = 0.0; 
        yawd = 0.0; 
        x_ << px, py, v, yaw, yawd;
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
    } else {
      timestep_++;
      Prediction(dt);
      UpdateLidar(meas_package);
    }
  }
  else if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_) {
    if (!is_initialized_) {
      /**
      Initialize state by radar measurement, 
      this involves converting radar measurements from polar to cartesian coordinates.
      */
      cout << "Initial radar measurement received!" << endl;
      float px, py, v, yaw, yawd;
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float rhodot = meas_package.raw_measurements_[2];
      px = rho * cos(phi);
      py = rho * sin(phi);
      v  = 0.0;
      yaw = 0.0; 
      yawd = 0.0; 
      x_ << px, py, v, yaw, yawd;
      
      // done initializing, no need to predict or update
      is_initialized_ = true;
    } else {
      timestep_++;
      Prediction(dt);
      UpdateRadar(meas_package);
    }
  }

  // print the output
  std::string sep = "\n----------------------------------------\n";
  Eigen::IOFormat CleanFmt(cout.precision(3), 0, ", ", "\n", "  [", "]");
  cout << "x_ = " << endl << x_.format(CleanFmt) << endl;
  cout << "P_ = " << endl << P_.format(CleanFmt) << endl << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
