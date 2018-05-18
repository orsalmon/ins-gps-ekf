//
// Created by Or Salmon on 18/05/18.
//

#include "NavigationState.h"

namespace EKF {
NavigationState::NavigationState() {
  NavigationState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                  Eigen::Matrix3d::Identity());
}

NavigationState::NavigationState(const Eigen::Vector3d p_0,
                                 const Eigen::Vector3d v_0,
                                 const Eigen::Matrix3d T_0)
    : p_n_(p_0.data()), v_n_(v_0.data()), T_bn_(T_0.data()) {}

void NavigationState::updateStateWithMeasurements(Eigen::Vector3d f_bi_b,
                                                  Eigen::Vector3d omega_bi_b) {
  p_dot_n_ = D() * v_n_;
  v_dot_n_ = T_bn_ * f_bi_b + g_n_ - (Omega_ne_n() + 2 * Omega_ei_n()) * v_n_;

  Eigen::Matrix3d Omega_bi_b;
  Utils::toSkewSymmetricMatrix(Omega_bi_b, omega_bi_b);
  T_dot_bn_ = T_bn_ * Omega_bi_b - (Omega_ei_n() + Omega_ne_n()) * T_bn_;
}

Eigen::Matrix3d NavigationState::D() {
  Eigen::Matrix3d D_ = Eigen::Matrix3d::Zero();

  D_(0, 0) = 1 / (Utils::getMeridianRadius(p_n_(phi)) + p_n_(h));
  D_(1, 1) =
      1 / (std::cos(p_n_(phi)) * (Utils::getNormalRadius(p_n_(phi)) + p_n_(h)));
  D_(2, 2) = -1;

  return D_;
}

Eigen::Matrix3d NavigationState::Omega_ne_n() {
  Eigen::Vector3d omega_ne_n_;
  omega_ne_n_(0) = v_n_(e) / (Utils::getNormalRadius(p_n_(phi)) + p_n_(h));
  omega_ne_n_(1) = -v_n_(n) / (Utils::getMeridianRadius(p_n_(phi)) + p_n_(h));
  omega_ne_n_(2) = -(v_n_(e) * std::tan(p_n_(phi))) /
                   (Utils::getNormalRadius(p_n_(phi)) + p_n_(h));

  Eigen::Matrix3d Omega_ne_n_;
  Utils::toSkewSymmetricMatrix(Omega_ne_n_, omega_ne_n_);

  return Omega_ne_n_;
}

Eigen::Matrix3d NavigationState::Omega_ei_n() {
  Eigen::Vector3d omega_ei_n_;
  omega_ei_n_(0) = omega_ei_ * std::cos(p_n_(phi));
  omega_ei_n_(1) = 0;
  omega_ei_n_(2) = -omega_ei_ * std::sin(p_n_(phi));

  Eigen::Matrix3d Omega_ei_n_;
  Utils::toSkewSymmetricMatrix(Omega_ei_n_, omega_ei_n_);

  return Omega_ei_n_;
}
} // namespace EKF
