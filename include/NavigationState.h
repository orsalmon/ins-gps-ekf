//
// Created by Or Salmon on 18/05/18.
//

#ifndef EKF_INS_NAVIGATIONSTATE_H
#define EKF_INS_NAVIGATIONSTATE_H

#include "Core.h"

namespace EKF_INS {
class NavigationState {
 public:
  NavigationState();
  void setState(const Eigen::Vector3d p, const Eigen::Vector3d v, const Eigen::Matrix3d T);
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> getState();
  void updateStateWithMeasurements(Eigen::Vector3d f_bi_b, Eigen::Vector3d omega_bi_b);
  void integrateState(double dt);

 private:
  Eigen::Matrix3d D();
  Eigen::Matrix3d Omega_ne_n();
  Eigen::Matrix3d Omega_ei_n();

  Eigen::Vector3d p_dot_n_, p_n_;
  Eigen::Vector3d v_dot_n_, v_n_;
  Eigen::Matrix3d T_dot_bn_, T_bn_;

  const Eigen::Vector3d g_n_; // TODO: can be more precise

  enum Pose { phi, lambda, h };
  enum Vel { n, e, d };
};
} // namespace EKF_INS

#endif // EKF_INS_NAVIGATIONSTATE_H
