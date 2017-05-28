#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  tools = Tools();
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;

  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 0, 0, 0, 0;

  ekf_.R_ = R_laser_;
  ekf_.H_ = H_laser_;

  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << 0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];

      float px = rho*cos(phi);
      float py = rho*sin(phi);
      if(fabs(px)<0.0001){
        px = 0.0001;
        ekf_.P_(0,0) = 1000;
      }
      if(fabs(py)<0.0001){
        py = 0.0001;
        ekf_.P_(1,1) = 1000;
      }
      ekf_.x_ << px, py, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];

      if(fabs(px)<0.0001){
        px = 0.0001;
        ekf_.P_(0,0) = 1000;
      }
      if(fabs(py)<0.0001){
        py = 0.0001;
        ekf_.P_(1,1) = 1000;
      }
      ekf_.x_ << px, py,0,0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;

    cout << "x_: \n" << ekf_.x_ << endl;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_ = MatrixXd(4,4);

  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1,  0,
             0, 0, 0,  1;
  
  ekf_.Q_ = MatrixXd(4, 4);

  float dt2 = dt  * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;

  ekf_.Q_ << dt4*9/4, 0      , dt3*9/2, 0      ,
             0      , dt4*9/4, 0      , dt3*9/2,
             dt3*9/2, 0      , dt2*9  , 0      ,
             0      , dt3*9/2, 0      , dt2*9  ;
  
  cout << "\nQ_:\n" << ekf_.Q_ << endl;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
