//
// Created by Or Salmon on 18/05/18.
//

#ifndef EKF_ERRORSTATE_H
#define EKF_ERRORSTATE_H

#include "Core.h"
#include "NavigationState.h"

namespace EKF {
class ErrorState {
public:
  ErrorState(std::shared_ptr<EKF::NavigationState> navigation_state_ptr);
  auto getState();
  void updateStateWithMeasurements(Eigen::Vector3d f_bi_b,
                                   Eigen::Vector3d omega_bi_b);
  void integrateState(double dt);

private:
  Eigen::Matrix3d Frr();
  Eigen::Matrix3d Frv();
  Eigen::Matrix3d Fre();
  Eigen::Matrix3d Fvr();
  Eigen::Matrix3d Fvv();
  Eigen::Matrix3d Fve();
  Eigen::Matrix3d Fer();
  Eigen::Matrix3d Fev();
  Eigen::Matrix3d Fee();
  Eigen::Matrix3d Fa();
  Eigen::Matrix3d Fg();
  void getNavigationState();

  Eigen::Vector3d delta_p_dot_n_, delta_p_n_;
  Eigen::Vector3d delta_v_dot_n_, delta_v_n_;
  Eigen::Vector3d epsilon_dot_n_, epsilon_n_;
  Eigen::Vector3d b_dot_a_, b_a_;
  Eigen::Vector3d b_dot_g_, b_g_;

  Eigen::Vector3d p_n_, v_n_;
  Eigen::Matrix3d T_bn_;
  Eigen::Vector3d f_bi_b_, omega_bi_b_;
  std::shared_ptr<EKF::NavigationState> navigation_state_ptr_;

  enum Pose { phi, lambda, h };
  enum Vel { n, e, d };
};
} // namespace EKF

#endif // EKF_ERRORSTATE_H
