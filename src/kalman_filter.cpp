#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // new state
  MatrixXd I = MatrixXd::Identity(4,4);
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;


}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // recover state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  // --- convert to polar coordinates ---
  double rho2 = px * px + py * py;
  // for the very unlikely case we have an ant having fun with our lidar...
  if(rho2 < 0.00001){
    px = 0.0001;
    py = 0.0001;
    rho2 = sqrt(px*px + py*py);
  }
  double rho = sqrt(rho2);
  double phi = atan2(py, px);
  double rhodot = (px * vx + py * vy) / rho;

  VectorXd z_pred(3);
  z_pred << rho, phi, rhodot;
  VectorXd y = z - z_pred;
  // normalize the angel to [-180, 180]
  y(1) = atan2(sin(y(1)), cos(y(1)));

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // new state
  MatrixXd I = MatrixXd::Identity(4,4);
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;


}
