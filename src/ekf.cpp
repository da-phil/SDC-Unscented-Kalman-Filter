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
  Eigen::IOFormat CleanFmt(cout.precision(3), 0, ", ", "\n", "  [", "]");

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
      cout << "x_ = " << endl << x_.format(CleanFmt) << endl;
      cout << "P_ = " << endl << P_.format(CleanFmt) << endl << endl;
      if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_)
        UpdateLidar(meas_package);
      if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_)
        UpdateRadar(meas_package);
  }

  if (verbose_) {
    // print the output
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

  // Vector / Matrix output format
  Eigen::IOFormat CleanFmt(cout.precision(3), 0, ", ", "\n", "  [", "]");
  double p_x, p_y, v, yaw, yawd;
  float dt_2 = delta_t * delta_t;
  float dt_3 = dt_2 * delta_t;
  float dt_4 = dt_3 * delta_t;

  p_x   = x_(0);
  p_y   = x_(1);
  v     = x_(2);
  yaw   = x_(3);
  yawd  = x_(4);

  VectorXd x_d  = VectorXd::Zero(n_x_);
  VectorXd u    = VectorXd::Zero(n_x_);
  MatrixXd Fj   = MatrixXd::Identity(n_x_, n_x_);
  MatrixXd Q    = MatrixXd::Zero(n_x_, n_x_);
  //predicted state values
  double px_p, py_p, v_p, yaw_p, yawd_p;

  //avoid division by zero
  if (fabs(yawd) > std::numeric_limits<double>::epsilon()) {
      px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t) );
  } else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
  }

  // add noise
  px_p += 0.5*std_a_*delta_t*delta_t * cos(yaw);
  py_p += 0.5*std_a_*delta_t*delta_t * sin(yaw);
  // prediction and adding noise
  v_p     = v + std_a_*delta_t;
  yaw_p   = yaw + yawd*delta_t + 0.5*std_yawdd_*delta_t*delta_t;
  yawd_p  = yawd + std_yawdd_*delta_t;

  x_d << px_p, py_p, v_p, yaw_p, yawd_p;

  u <<  0.5*dt_2*cos(yaw)*std_a_,
        0.5*dt_2*sin(yaw)*std_a_,
        delta_t*std_a_,
        0.5*dt_2*std_yawdd_,
        delta_t*std_yawdd_;

  x_ += x_d + u;
  x_(3) = fmod(x_(3), 2. * M_PI);
  
  // F matrix is jacobian matrix Fj
  if (fabs(yawd) < std::numeric_limits<double>::epsilon()) {
    yawd = std::numeric_limits<double>::epsilon();
  }
  double Fj02 = v/yawd * (cos(delta_t*yawd + yaw) - cos(yaw));
  double Fj03 = 1/yawd * (sin(delta_t*yawd + yaw) - sin(yaw));
  double Fj04 = delta_t*v/yawd * cos(delta_t*yawd + yaw) - v/(yawd*yawd)*(sin(delta_t*yawd + yaw) - sin(yaw));
  double Fj12 = v/yawd * (sin(delta_t*yawd + yaw) - sin(yaw));
  double Fj13 = 1/yawd * (cos(yaw) - cos(delta_t*yawd + yaw));
  double Fj14 = delta_t*v/yawd * sin(delta_t*yawd + yaw) - v/(yawd*yawd)*(cos(yaw) - cos(delta_t*yawd + yaw));
  Fj << 1.,  0.,  Fj02, Fj03, Fj04,
        0.,  1.,  Fj12, Fj13, Fj14,
        0.,  0.,  0.,    1.,    0.,
        0.,  0.,  1.,    0.,    delta_t,
        0.,  0.,  0.,    0.,    1.;

  double q1 = 0.25*dt_4*std_a_*std_a_;
  double q2 = 0.5*dt_3*std_a_*std_a_;
  double q3 = dt_2*std_a_*std_a_;
  double q4 = 0.25*dt_4*std_yawdd_*std_yawdd_;
  double q5 = 0.5*dt_3*std_yawdd_*std_yawdd_;
  double q6 = dt_2*std_yawdd_*std_yawdd_;
  Q <<  q1,  q1,  q2,  0,  0,
        q1,  q1,  q2,  0,  0,
        q2,  q2,  q3,  0,  0,
        0,   0,   0,   q4, q5,
        0,   0,   0,   q5, q6;


  P_ = Fj * P_ * Fj.transpose() + Q;
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

  nis_laser_ = y.transpose() * Si * y;
  if (nis_laser_ > CHI_SQ_2)
    nis_laser_counter_++;

  if (verbose_) {
    cout << "NIS(laser): ";
    cout << 100.0 * nis_laser_counter_ / timestep_ << "% (" << nis_laser_counter_ << " samples out of " << timestep_ << ") are out of 95% NIS range!" << endl;
  }
}



/**
* Measurement equation for radar update
*/
MatrixXd EKF::h_radar(void) {
  VectorXd z_pred = VectorXd(3);
  double px, py, v, yaw, yawd;
  px = x_(0);
  py = x_(1);
  v = x_(2);
  yaw = x_(3);
  yawd = x_(4);

  z_pred(0) = sqrt(px*px + py*py);

  if (fabs(px) <= std::numeric_limits<float>::epsilon())
    px = std::numeric_limits<float>::epsilon();
  if (fabs(py) <= std::numeric_limits<float>::epsilon())
    py = std::numeric_limits<float>::epsilon();

  z_pred(1) = fmod(atan2(py, px), 2.*M_PI);
  z_pred(2) = (px*v*cos(yaw) + py*v*sin(yaw)) / z_pred(0);

  return z_pred;
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
  double px, py, v, yaw, yawd, norm, px_2, py_2, H11, H12, H21, H22, H31, H32, H34;

  px = x_(0);
  py = x_(1);
  v = x_(2);
  yaw = x_(3);
  yawd = x_(4);
  px_2 = px*px;
  py_2 = py*py;
  norm = sqrt(px_2 + py_2);

  H11 = px / norm;
  H12 = py / norm;
  H21 = - py / (px_2 * (1. + py_2 / px_2));
  H22 = 1. / (px * (1. + py_2 / px_2));
  H31 = v * cos(yaw) / (norm - px/pow(norm, 3)) * (v*px*cos(yaw) + v*py*sin(yaw));
  H32 = v * sin(yaw) / (norm - py/pow(norm, 3)) * (v*px*cos(yaw) + v*py*sin(yaw));
  H34 = 1. / norm * (px * cos(yaw) + py * sin(yaw));
  H_radar_ << H11,   H12,    0,    0,      0,
              H21,   H22,    0,    0,      0,
              H31,   H32,    0,    H34,    0;

  // don't update measurement if we can't compute the jacobian
  if (H_radar_.isZero(0)){
    cerr << "H matrix of radar is zero" << endl;
    return;
  }

  // residual
  VectorXd z_diff = meas_package.raw_measurements_ - h_radar();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  MatrixXd H_radar_t = H_radar_.transpose();
  MatrixXd S = H_radar_ * P_ * H_radar_t + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * H_radar_t;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * z_diff);
  P_ = (I - K * H_radar_) * P_;

  nis_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  if (nis_radar_ > CHI_SQ_3)
    nis_radar_counter_++;

  if (verbose_) {
    cout << "NIS(radar): ";
    cout << 100.0 * nis_radar_counter_ / timestep_ << "% (" << nis_radar_counter_ << " samples out of " << timestep_ << ") are out of 95% NIS range!" << endl;
  }
}
