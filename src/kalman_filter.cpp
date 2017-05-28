#include "kalman_filter.h"
#include <cmath>
#include <math.h>
#include <iostream>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

//Correct
void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  VectorXd z_pred = H_*x_;
  VectorXd y = z-z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_*Ht*Si;
  x_ = x_ + K*y;
  P_ = (I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  VectorXd y = z - getH_x(x_);
  y[1] = pi_to_minus_pi(y[1]);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_*Ht*Si;

  x_ = x_ + K*y;
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  P_ = (I - K*H_)*P_;  

}

float KalmanFilter::pi_to_minus_pi(float x){
  // return result;
  float min = -M_PI;
  float max = M_PI;
  /* wrap x -> [0,max) */
  float wrapMax = fmod((max-min) + fmod((x-min), (max-min)), (max-min));
  return  min + wrapMax;
}



VectorXd KalmanFilter::getH_x(const VectorXd &x){
  VectorXd h_x = VectorXd(3);
  float px = x[0];
  float py = x[1];
  float vx = x[2];
  float vy = x[3];

  float px2 = px*px;
  float py2 = py*py;
  float r = sqrt(px2 + py2);

  if(r < 0.0001){
    cout << "Error :: Division by zero " << endl;
    r = 0.0001;
  }else{
    if(fabs(px) < 0.0001){
      cout << "Error :: Division by zero " << endl;
      px = 0.0001;
    }
    float numerator = (px*vx + py*vy);
    float phi = pi_to_minus_pi(atan2(py,px));
    float r_dot = numerator/r;
    h_x << r, phi, r_dot;
  }

  return h_x;
}
