//
// Created by Or Salmon on 18/05/18.
//

#ifndef EKF_INS_TRACKING_H
#define EKF_INS_TRACKING_H

#include "Core.h"
#include "ErrorState.h"
#include "ErrorStateCovariance.h"
#include "NavigationState.h"
#include <chrono>
#include <numeric>
#include <vector>

namespace EKF_INS {
class EKF;

class Tracking {
  friend class EKF;
public:
  Tracking(bool use_azimuth_alignment);
  void updateTrackingWithAccelerometer(Eigen::Vector3d f_bi_b);
  void updateTrackingWithGyro(Eigen::Vector3d omega_bi_b);
  void setNavigationInitialState(const Eigen::Vector3d p,
                                 const Eigen::Vector3d v,
                                 const Eigen::Matrix3d T);
  auto getNavigationState() { return navigation_state_ptr_->getState(); }
  Eigen::VectorXd getErrorState() { return error_state_ptr_->getState(); }
  Eigen::MatrixXd getErrorStateCovariance() {
    return error_state_covariance_ptr_->getErrorStateCovariance();
  }

 protected:
  void resetClock();
  void setQMatrix(Eigen::MatrixXd Q);

private:
  void updateTrackingWithMeasurements(Eigen::Vector3d f_bi_b, Eigen::Vector3d omega_bi_b);
  void checkAndUpdate();
  void updateDT();

  std::shared_ptr<EKF_INS::NavigationState> navigation_state_ptr_;
  std::shared_ptr<EKF_INS::ErrorState> error_state_ptr_;
  std::shared_ptr<EKF_INS::ErrorStateCovariance> error_state_covariance_ptr_;

  std::vector<Eigen::Vector3d> f_measurments_, g_measurments_;
  std::chrono::duration<double> dt_;
  std::chrono::_V2::system_clock::time_point clock_;

  std::shared_ptr<spdlog::logger> logger_;

  const bool use_azimuth_alignment_;
};
} // namespace EKF_INS

#endif // EKF_INS_TRACKING_H
