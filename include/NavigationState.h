//
// Created by Or Salmon on 18/05/18.
//

#ifndef EKF_NAVIGATIONSTATE_H
#define EKF_NAVIGATIONSTATE_H

#include "Core.h"

namespace EKF {
class NavigationState {
public:
  NavigationState();
  NavigationState(const Eigen::Vector3d p_0, const Eigen::Vector3d v_0,
                  const Eigen::Matrix3d T_0);
  void updateStateWithMeasurements(Eigen::Vector3d f_bi_b, Eigen::Vector3d omega_bi_b);
  void integrateState();

private:
  Eigen::Matrix3d D();
  Eigen::Matrix3d Omega_ne_n();
  Eigen::Matrix3d Omega_ei_n();

  Eigen::Vector3d p_dot_n_, p_n_;
  Eigen::Vector3d v_dot_n_, v_n_;
  Eigen::Matrix3d T_dot_bn_, T_bn_;

  const Eigen::Vector3d g_n_ = {0,0,9.81}; //TODO: can be more precise
  const double omega_ei_ = 7.292E-05; // Earth turn rate [rad/s]

  enum Pose {
    phi,
    lambda,
    h
  };

  enum Vel {
    n,
    e,
    d
  };
};
} // namespace EKF

#endif // EKF_NAVIGATIONSTATE_H
