#include "ukf.h"
#include <Eigen/Dense>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


UKF::UKF() {
  Init(false, true, true, 0.5, 0.2);
}


UKF::UKF(bool verbose, bool use_laser, bool use_radar, double std_a, double std_yawdd) {
  Init(verbose, use_laser, use_radar, std_a, std_yawdd);
}


/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
void UKF::Init(bool verbose, bool use_laser, bool use_radar,
               double std_a, double std_yawdd)
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = use_laser;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = use_radar;

  verbose_ = verbose;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  R_lidar_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);

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
  n_x_        = 5;
  n_aug_      = 7;
  lambda_     = 3-n_aug_;
  time_us_    = 0;
  weights_    = VectorXd::Zero(2*n_aug_+1);
  x_          = VectorXd::Zero(n_x_);
  Xsig_pred_  = MatrixXd::Zero(n_x_, 2*n_aug_+1);
  P_          = MatrixXd::Identity(n_x_, n_x_);
  R_radar_ <<   std_radr_*std_radr_,  0,                          0,
                0,                    std_radphi_*std_radphi_,    0,
                0,                    0,                          std_radrd_*std_radrd_;
  R_lidar_ <<   std_laspx_*std_laspx_,  0,
                0,                      std_laspy_*std_laspy_;

  //set weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2*n_aug_+1 ; i++)
    weights_(i) = 1 / (2*(lambda_+n_aug_));

  nis_laser_counter_ = 0;
  nis_radar_counter_ = 0;
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
  double dt;
 
  //compute the time elapsed between the current and previous measurements
  dt = (meas_package.timestamp_ - time_us_) / 1.0e6; //time in seconds
  time_us_ = meas_package.timestamp_;
  cout << "Timestep " << timestep_ << " - dt: " << dt << endl;
  timestep_++;

  if (!is_initialized_) {
    if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_) {
      /**
      Initialize state by lidar measurement.
      */
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
  if (verbose_)
    cout << "Prediction step" << endl;

    //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  // Vector / Matrix output format
  Eigen::IOFormat CleanFmt(cout.precision(3), 0, ", ", "\n", "  [", "]");

  //create augmented mean state
  x_aug << x_, 0, 0;

  //create augmented covariance matrix
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++) {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
  

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++) {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p, v_p, yaw_p, yawd_p;

    //avoid division by zero
    if (fabs(yawd) > std::numeric_limits<double>::epsilon()) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    // add noise
    px_p += 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p += 0.5*nu_a*delta_t*delta_t * sin(yaw);
    // prediction and adding noise
    v_p     = v + nu_a*delta_t;
    yaw_p   = yaw + yawd*delta_t + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p  = yawd + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_.col(i) << px_p, py_p, v_p, yaw_p, yawd_p;
  }


  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)  //iterate over sigma points
    x_ += weights_(i) * Xsig_pred_.col(i);

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    x_diff(3) = fmod(x_diff(3), 2.0*M_PI);
    P_ += weights_(i) * x_diff * x_diff.transpose() ;
  }

  if (verbose_) {
    cout << "Xsig_pred_: " << endl << Xsig_pred_.format(CleanFmt) << endl;
    cout << "x_: " << endl << x_.format(CleanFmt) << endl;
    cout << "P_: " << endl << P_.format(CleanFmt) << endl;
  }
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
  if (verbose_)
    cout << "UpdateLidar step" << endl;

 // Vector / Matrix output format
  Eigen::IOFormat CleanFmt(cout.precision(3), 0, ", ", "\n", "  [", "]");
  int n_z = 2; // measurement dimension
  double p_x, p_y, nis;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
  //mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  // measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);

  //transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_+1; i++) {  //2n+1 simga points
    // measurement model
    Zsig.col(i) << Xsig_pred_(0,i), Xsig_pred_(1,i);  // px, py
  }

  //mean predicted measurement
  z_pred = Zsig * weights_;
  // measurement residual
  MatrixXd Z_diff = Zsig.colwise() - z_pred;
  // state difference
  MatrixXd X_diff = Xsig_pred_.colwise() - x_;

  // innovation covariance matrix S
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //angle normalization
    Z_diff(1, i) = fmod(Z_diff(1,i), 2.0*M_PI);
    S += weights_(i) * Z_diff.col(i) * Z_diff.col(i).transpose();
  }
  // add measurement noise
  S += R_lidar_;

  // calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //angle normalization
    X_diff(3, i) = fmod(X_diff(3, i), 2.0*M_PI);
    Tc += weights_(i) * X_diff.col(i) * Z_diff.col(i).transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  //angle normalization
  z_diff(1) = fmod(z_diff(1), 2.0*M_PI);
  
  //update state mean and covariance matrix
  x_  += K * z_diff;
  P_  -= K * S * K.transpose();

  nis = z_diff.transpose() * S.inverse() * z_diff;
  nis_laser_.push_back(nis);
  cout << "NIS(laser): ";
  if (verbose_) {
    cout << endl;
    write_vec(nis_laser_);
    cout << endl;
  }

  if (nis > 5.991)
    nis_laser_counter_++;

  cout << 100.0 * nis_laser_counter_ / timestep_ << "% (" << nis_laser_counter_ << " samples out of " << timestep_ << ") are out of 95% NIS range!" << endl;

  if (verbose_) {
    cout << "x_: " << endl << x_.format(CleanFmt) << endl;
    cout << "Xsig_pred_: " << endl << Xsig_pred_.format(CleanFmt) << endl;
    cout << "S: " << endl << S.format(CleanFmt) <<  endl;
    cout << "Tc: " << endl << Tc.format(CleanFmt) << endl;
  }
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
  if (verbose_)
    cout << "UpdateRadar step" << endl;

  // Vector / Matrix output format
  Eigen::IOFormat CleanFmt(cout.precision(3), 0, ", ", "\n", "  [", "]");
  int n_z = 3; // measurement dimension
  double p_x, p_y, v, yaw, nis;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
  //mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  // measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);

  //transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_+1; i++) {  //2n+1 simga points
    // extract values for better readibility
    p_x = Xsig_pred_(0,i);
    p_y = Xsig_pred_(1,i);
    v   = Xsig_pred_(2,i);
    yaw = Xsig_pred_(3,i);

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);              //r
    Zsig(1,i) = atan2(p_y,p_x);                       //phi
    Zsig(2,i) = (p_x*cos(yaw)*v + p_y*sin(yaw)*v) / Zsig(0,i);       //r_dot
  }

  //mean predicted measurement
  z_pred = Zsig * weights_;
  // measurement residual
  MatrixXd Z_diff = Zsig.colwise() - z_pred;
  // state difference
  MatrixXd X_diff = Xsig_pred_.colwise() - x_;

  // innovation covariance matrix S
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //angle normalization
    Z_diff(1, i) = fmod(Z_diff(1,i), 2.0*M_PI);
    S += weights_(i) * Z_diff.col(i) * Z_diff.col(i).transpose();
  }
  // add measurement noise
  S += R_radar_;

  // calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //angle normalization
    X_diff(3, i) = fmod(X_diff(3, i), 2.0*M_PI);
    Tc += weights_(i) * X_diff.col(i) * Z_diff.col(i).transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  //angle normalization
  z_diff(1) = fmod(z_diff(1), 2.0*M_PI);
  
  //update state mean and covariance matrix
  x_  += K * z_diff;
  P_  -= K * S * K.transpose();

  nis = z_diff.transpose() * S.inverse() * z_diff;
  if (nis > 7.8)
    nis_radar_counter_++;

  nis_radar_.push_back(nis);

  cout << "NIS(radar): ";
  if (verbose_) {
    cout << endl;
    write_vec(nis_radar_);
    cout << endl;
  }

  cout << 100.0 * nis_radar_counter_ / timestep_ << "% (" << nis_radar_counter_ << " samples out of " << timestep_ << ") are out of 95% NIS range!" << endl;

  if (verbose_) {
    cout << "x_: " << endl << x_.format(CleanFmt) << endl;
    cout << "Xsig_pred_: " << endl << Xsig_pred_.format(CleanFmt) << endl;
    cout << "S: " << endl << S.format(CleanFmt) <<  endl;
    cout << "Tc: " << endl << Tc.format(CleanFmt) << endl;
  }
}

void UKF::write_vec(const vector<double>& vec) {
    for (vector<double>::const_iterator iter = vec.begin();
        iter != vec.end(); ++iter) {
        cout << *iter << ", ";
    }
}
