#ifndef EKF_H
#define EKF_H

#include "measurement_package.h"
#include "filter.h"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class EKF: public Filter {
public:

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
  * Measurement equation for radar update
  */
  MatrixXd h_radar(void);


  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
};

#endif /* EKF_H */
