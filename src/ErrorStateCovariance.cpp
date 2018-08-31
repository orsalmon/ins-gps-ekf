//
// Created by Or Salmon on 20/06/18.
//

#include "ErrorStateCovariance.h"

namespace EKF_INS {
ErrorStateCovariance::ErrorStateCovariance(std::shared_ptr<EKF_INS::ErrorState> error_state_ptr) : P_(15, 15), Q_(15, 15) {
  error_state_ptr_ = error_state_ptr;
  P_.setZero();
  Q_.setZero();
}

ErrorStateCovariance::ErrorStateCovariance(std::shared_ptr<EKF_INS::ErrorState> error_state_ptr, Eigen::MatrixXd Q) : ErrorStateCovariance(
    error_state_ptr) {
  setQMatrix(Q);
}

void ErrorStateCovariance::setQMatrix(Eigen::MatrixXd Q) { Q_ = Q; }

void ErrorStateCovariance::updateCovarianceMatrix(double dt) {
  Eigen::MatrixXd phi = error_state_ptr_->getTransitionMatrix(dt);
  P_ = phi * P_ * phi.transpose() + Q_;
}

Eigen::MatrixXd ErrorStateCovariance::getErrorStateCovariance() { return P_; }

void ErrorStateCovariance::resetPMatrix() {
  P_.setZero();
}
} // namespace EKF_INS
