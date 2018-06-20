//
// Created by Or Salmon on 18/05/18.
//

#ifndef EKF_INS_TRACKING_H
#define EKF_INS_TRACKING_H

#include "Core.h"
#include "ErrorState.h"
#include "NavigationState.h"
#include "ErrorStateCovariance.h"

namespace EKF_INS {
class Tracking {
public:
  Tracking();
  void updateTrackingWithMeasurements(Eigen::Vector3d f_bi_b,
                                      Eigen::Vector3d omega_bi_b);
  void setDeltaT(double dt) { dt_ = dt; }
  void setNavigationInitialState(const Eigen::Vector3d p,
                                 const Eigen::Vector3d v,
                                 const Eigen::Matrix3d T);
  auto getNavigationState() { return navigation_state_ptr_->getState(); }
  Eigen::VectorXd getErrorState() { return error_state_ptr_->getState(); }

private:
  std::shared_ptr<EKF_INS::NavigationState> navigation_state_ptr_;
  std::shared_ptr<EKF_INS::ErrorState> error_state_ptr_;
  std::shared_ptr<EKF_INS::ErrorStateCovariance> error_state_covariance_ptr_;
  double dt_;
};
} // namespace EKF_INS

#endif // EKF_INS_TRACKING_H
