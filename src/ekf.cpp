#include "ekf.h"
#include <Eigen/Dense>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Chi-square distribution parameters for NIS check
#define CHI_SQ_3  7.8
#define CHI_SQ_2  5.991


EKF::EKF() {
  Init();
}


EKF::EKF(bool verbose, bool use_laser, bool use_radar, double std_a, double std_yawdd) {
  Init(verbose, use_laser, use_radar, std_a, std_yawdd);
}


/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
void EKF::Init(bool verbose, bool use_laser, bool use_radar, double std_a, double std_yawdd)
{
  n_x_        = 5; // we use 5 state variables: px, py, v, yaw, yawrate
  n_aug_      = 7; // augmented state vector additionally includes std_a and std_yawdd

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = use_laser;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = use_radar;
  verbose_ = verbose;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  H_laser_ = MatrixXd(2, n_x_);
  H_radar_ = MatrixXd(3, n_x_);

  R_lidar_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);

  // measurement matrix for linear kalman filter update from laser scanner data
  H_laser_ << 1,    0,    0,    0,  0,
              0,    1,    0,    0,  0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = std_a;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = std_yawdd;
  
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
  time_us_    = 0;
  x_          = VectorXd::Zero(n_x_);
  P_          = MatrixXd::Identity(n_x_, n_x_);
  R_radar_ <<   std_radr_*std_radr_,  0,                          0,
                0,                    std_radphi_*std_radphi_,    0,
                0,                    0,                          std_radrd_*std_radrd_;
  R_lidar_ <<   std_laspx_*std_laspx_,  0,
                0,                      std_laspy_*std_laspy_;

  nis_laser_counter_ = 0;
  nis_radar_counter_ = 0;
}


EKF::~EKF() {}


/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void EKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  double dt;
 
  //compute the time elapsed between the current and previous measurements
  dt = (meas_package.timestamp_ - time_us_) / 1.0e6; //time in seconds
  time_us_ = meas_package.timestamp_;
  if (verbose_)
    cout << "Timestep " << timestep_ << " - dt: " << dt << endl;
  timestep_++;

  if (!is_initialized_) {
    if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_) {
      /**
      Initialize state by lidar measurement.
      */
      if (verbose_)
        cout << "Initial lidar measurement received!" << endl;

      double px, py, v, yaw, yawd;
      px = meas_package.raw_measurements_[0];
      py = meas_package.raw_measurements_[1];
      v  = 0.0;
      yaw = 0.0; 
      yawd = 0.0; 
      x_ << px, py, v, yaw, yawd;

      // done initializing, no need to predict or update
      is_initialized_ = true;
    } 
    if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_) {
      /**
      Initialize state by radar measurement, 
      this involves converting radar measurements from polar to cartesian coordinates.
      */
      if (verbose_)
          cout << "Initial radar measurement received!" << endl;

      double px, py, v, yaw, yawd;
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rhodot = meas_package.raw_measurements_[2];
      px = rho * cos(phi);
      py = rho * sin(phi);
      v  = 0.0;
      yaw = 0.0; 
      yawd = 0.0; 
      x_ << px, py, v, yaw, yawd;

      // done initializing, no need to predict or update
      is_initialized_ = true;
    }
  }
  else {
      Prediction(dt);

      if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_)
        UpdateLidar(meas_package);
      if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_)
        UpdateRadar(meas_package);
  }

  if (verbose_) {
    // print the output
    std::string sep = "\n----------------------------------------\n";
    Eigen::IOFormat CleanFmt(cout.precision(3), 0, ", ", "\n", "  [", "]");
    cout << "x_ = " << endl << x_.format(CleanFmt) << endl;
    cout << "P_ = " << endl << P_.format(CleanFmt) << endl << endl;
  }
}

/**
 * Predicts the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void EKF::Prediction(double delta_t) {
  if (verbose_)
    cout << "Prediction step" << endl;

    //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  // Vector / Matrix output format
  Eigen::IOFormat CleanFmt(cout.precision(3), 0, ", ", "\n", "  [", "]");

  //create augmented mean state
  x_aug << x_, 0, 0;

  //create augmented covariance matrix
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;



  // x_ = ???
  // P_ = ???
}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * This function just uses the simple linear Kalman filter equations becasuse
 * the measurement equations are linear (we measure the position states directly).
 * @param {MeasurementPackage} meas_package
 */
void EKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  if (verbose_)
    cout << "UpdateLidar step" << endl;

  VectorXd y = meas_package.raw_measurements_ - H_laser_ * x_;
  MatrixXd Ht = H_laser_.transpose();
  MatrixXd S = H_laser_ * P_ * Ht + R_lidar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I - K * H_laser_) * P_;

  nis_laser_ = y.transpose() * S.inverse() * y;
  if (nis_laser_ > CHI_SQ_2)
    nis_laser_counter_++;

  if (verbose_) {
    cout << "NIS(laser): ";
    cout << 100.0 * nis_laser_counter_ / timestep_ << "% (" << nis_laser_counter_ << " samples out of " << timestep_ << ") are out of 95% NIS range!" << endl;
  }
}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void EKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  if (verbose_)
    cout << "UpdateRadar step" << endl;

  // Vector / Matrix output format
  Eigen::IOFormat CleanFmt(cout.precision(3), 0, ", ", "\n", "  [", "]");
  int n_z = 3; // measurement dimension

  //mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  // measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);





  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  //angle normalization
  z_diff(1) = fmod(z_diff(1), 2.0*M_PI);
  
  //update state mean and covariance matrix
  x_  += K * z_diff;
  P_  -= K * S * K.transpose();

  nis_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  if (nis_radar_ > CHI_SQ_3)
    nis_radar_counter_++;

  if (verbose_) {
    cout << "NIS(radar): ";
    cout << 100.0 * nis_radar_counter_ / timestep_ << "% (" << nis_radar_counter_ << " samples out of " << timestep_ << ") are out of 95% NIS range!" << endl;
  }
}
