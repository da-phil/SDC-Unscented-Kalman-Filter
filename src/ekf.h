#ifndef EKF_H
#define EKF_H

#include "measurement_package.h"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class EKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  //measurement matrix for linear kalman filter update from laser scanner data
  MatrixXd H_laser_;

  // jacobian containing the measurement matrix for radar measurements
  MatrixXd H_radar_;

  // Measurement noise matricies
  MatrixXd R_lidar_;
  MatrixXd R_radar_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  //* Timestep count in simualtion
  int timestep_;

  //* Verbose flag for activating extra debug output
  bool verbose_;

  //* Normalized Innovation Squared (NIS) value for laser and radar
  double nis_laser_;
  double nis_radar_;
  int nis_laser_counter_;
  int nis_radar_counter_;

  /**
   * Constructor
   */
  EKF();
  EKF(bool verbose, bool use_laser, bool use_radar, double std_a, double std_yawdd);
  /**
   * Destructor
   */
  virtual ~EKF();

  /**
   * EKF initialization
   */
  void Init(bool verbose = false, bool use_laser = true, bool use_radar = true,
            double std_a = 2.0, double std_yawdd = 0.5);

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement.
   * This function just uses the simple linear Kalman filter equations becasuse
   * the measurement equations are linear (we measure the position states directly).
   * @param {MeasurementPackage} meas_package
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
};

#endif /* EKF_H */
