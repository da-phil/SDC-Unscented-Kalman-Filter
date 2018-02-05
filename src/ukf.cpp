#include "ukf.h"
#include <Eigen/Dense>
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
  n_x_        = 5;
  n_aug_      = 7;
  lambda_     = 3-n_aug_;
  time_us_    = 0;
  weights_    = VectorXd::Zero(2*n_aug_+1);
  x_          = VectorXd::Zero(n_x_);
  Xsig_pred_  = MatrixXd::Zero(n_x_, 2*n_aug_+1);
  P_          = MatrixXd::Identity(n_x_, n_x_);

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
      cout << "Timestep " << timestep_ << " - dt: " << dt << " - Lidar measurement" << endl;
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
      cout << "Timestep " << timestep_ << " - dt: " << dt << " - Radar measurement" << endl;
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

  // augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  // augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  // create sigma point matrix of augmented state space
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  double v, yaw, yawd, nu_a, nu_yawdd;

  //create augmented mean state
  x_aug << x_, 0, 0;

  //create augmented covariance matrix
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.block(n_x_, n_x_, 2, 2) <<  std_a_*std_a_,  0,
                                    0,              std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug.col(0) << x_aug;
  Xsig_aug.block(0, 1, n_aug_, n_aug_) = (sqrt(lambda_+n_aug_) * A).colwise() + x_aug;
  Xsig_aug.block(0, n_aug_+1, n_aug_, n_aug_) = (-sqrt(lambda_+n_aug_) * A).colwise() + x_aug;


  //predicted state values by applying kinematics propagation function f(x_aug)
  VectorXd x_pred_k = VectorXd::Zero(n_x_);
  double delta_t_squared = delta_t*delta_t;
  
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    v     = Xsig_aug(2,i);
    yaw   = Xsig_aug(3,i);
    yawd  = Xsig_aug(4,i);
    nu_a  = Xsig_aug(5,i);
    nu_yawdd = Xsig_aug(6,i);

    //avoid division by zero
    if (fabs(yawd) > std::numeric_limits<double>::epsilon()) {
        x_pred_k(0) += v/yawd * (sin(yaw+yawd*delta_t) - sin(yaw));
        x_pred_k(1) += v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
    } else {
        x_pred_k(0) += v*delta_t*cos(yaw);
        x_pred_k(1) += v*delta_t*sin(yaw);
    }

    // add noise on position
    x_pred_k(0) += 0.5*nu_a*delta_t_squared * cos(yaw);
    x_pred_k(1) += 0.5*nu_a*delta_t_squared * sin(yaw);
    // velocity, yaw and yawdot including noise
    x_pred_k(2) += nu_a*delta_t;
    x_pred_k(3) += yawd*delta_t + 0.5*nu_yawdd*delta_t_squared;
    x_pred_k(4) += nu_yawdd*delta_t;
    //write predicted sigma point into right column
    Xsig_pred_.col(i) = x_pred_k;
  }

  MatrixXd Xsig_pred_meanfree = Xsig_pred_;
  
  //set weights
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i = 1; i < 2*n_aug_+1 ; i++)
    weights_(i) = 1 / (2*(lambda_+n_aug_));

  //predicted state mean
  x_ = Xsig_pred_ * weights_;
  
  //predicted state covariance matrix
  Xsig_pred_meanfree.colwise() -= x_;
  // make sure yaw stays in range -2*PI  to  +2*PI
  Xsig_pred_meanfree(3) = fmod(Xsig_pred_meanfree(3), 2.0*M_PI);
  
  P_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++)  //iterate over sigma points
    P_ += weights_(i) * Xsig_pred_meanfree.col(i) * Xsig_pred_meanfree.col(i).transpose();
  
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
