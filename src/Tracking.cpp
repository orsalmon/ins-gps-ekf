//
// Created by or-caja on 18/05/18.
//

#include "Tracking.h"

namespace EKF_INS {
Tracking::Tracking() : dt_(0) {
  navigation_state_ptr_.reset(new EKF_INS::NavigationState());
  error_state_ptr_.reset(new EKF_INS::ErrorState(navigation_state_ptr_));
}

void Tracking::updateTrackingWithMeasurements(Eigen::Vector3d f_bi_b,
                                              Eigen::Vector3d omega_bi_b) {
  navigation_state_ptr_->updateStateWithMeasurements(f_bi_b, omega_bi_b);
  navigation_state_ptr_->integrateState(dt_);
  error_state_ptr_->updateStateWithMeasurements(f_bi_b, omega_bi_b);
  error_state_ptr_->integrateState(dt_);
}

void Tracking::setNavigationInitialState(const Eigen::Vector3d p,
                                         const Eigen::Vector3d v,
                                         const Eigen::Matrix3d T) {
  navigation_state_ptr_->setState(p, v, T);
}
} // namespace EKF_INS
