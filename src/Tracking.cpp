//
// Created by Or Salmon on 18/05/18.
//

#include "Tracking.h"

namespace EKF_INS {
Tracking::Tracking() {
  navigation_state_ptr_.reset(new EKF_INS::NavigationState());
  error_state_ptr_.reset(new EKF_INS::ErrorState(navigation_state_ptr_));
  error_state_covariance_ptr_.reset(
      new EKF_INS::ErrorStateCovariance(error_state_ptr_));

  resetClock();
}

void Tracking::updateTrackingWithAccelerometer(Eigen::Vector3d f_bi_b) {
  f_measurments_.push_back(f_bi_b);
  checkAndUpdate();
}

void Tracking::updateTrackingWithGyro(Eigen::Vector3d omega_bi_b) {
  g_measurments_.push_back(omega_bi_b);
  checkAndUpdate();
}

void Tracking::updateTrackingWithMeasurements(Eigen::Vector3d f_bi_b,
                                              Eigen::Vector3d omega_bi_b) {
  navigation_state_ptr_->updateStateWithMeasurements(f_bi_b, omega_bi_b);
  navigation_state_ptr_->integrateState(dt_.count());
  error_state_ptr_->updateStateWithMeasurements(f_bi_b, omega_bi_b);
  error_state_ptr_->integrateState(dt_.count());
}

void Tracking::setNavigationInitialState(const Eigen::Vector3d p,
                                         const Eigen::Vector3d v,
                                         const Eigen::Matrix3d T) {
  navigation_state_ptr_->setState(p, v, T);
}

void Tracking::resetClock() {
  clock_ = std::chrono::high_resolution_clock::now();
}

void Tracking::checkAndUpdate() {
  if (!f_measurments_.empty() && !g_measurments_.empty()) {
    Eigen::Vector3d f_mean = Utils::calcMeanVector(f_measurments_);
    Eigen::Vector3d g_mean = Utils::calcMeanVector(g_measurments_);
    f_measurments_.clear();
    g_measurments_.clear();
    updateDT();
    updateTrackingWithMeasurements(f_mean, g_mean);
    resetClock();
  }
}

void Tracking::updateDT() {
  dt_ = std::chrono::high_resolution_clock::now() - clock_;
}

void Tracking::setQMatrix(Eigen::MatrixXd Q) {
  error_state_covariance_ptr_->setQMatrix(Q);
}
} // namespace EKF_INS
