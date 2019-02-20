#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  UpdateEstimate(y);
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   

  double ro     = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
  // atan2 gives angle between -pi and pi 
  double phi    = atan2(x_[1], x_[0]);
  double rodot;
  // checking for zero 
  if (ro==0) {
    return;   
  } 
  rodot = (x_[0]*x_[2] + x_[1]*x_[3])/ro;
  
  VectorXd z_pred(3);
  z_pred << ro,phi,rodot;
  
  VectorXd y = z - z_pred;

  //cout << "phi before normalization " << y[1] << endl;
  // Normalizing angle to be within -Pi to +Pi 
  while ( y[1] > M_PI || y[1] < -M_PI ) {
    if(y[1] < -(M_PI)){
    y[1] = y[1] + (2*M_PI);
  }
  if(y[1] > M_PI){
    y[1] = y[1] - (2*M_PI);
  }
}
  
  //cout << "phi after normalization " << y[1] << endl;

  UpdateEstimate(y);

}

void KalmanFilter::UpdateEstimate(const VectorXd &y) {

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}




