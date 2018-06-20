//
// Created by Or Salmon on 18/05/18.
//

#include "NavigationState.h"

namespace EKF_INS {
NavigationState::NavigationState() : g_n_({0, 0, 1*Utils::g}) {
  setState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
           Eigen::Matrix3d::Identity());
}

void NavigationState::setState(const Eigen::Vector3d p, const Eigen::Vector3d v,
                               const Eigen::Matrix3d T) {
  p_n_ = p;
  v_n_ = v;
  T_bn_ = T;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>
NavigationState::getState() {
  return std::make_tuple(p_n_, v_n_, T_bn_);
}

void NavigationState::updateStateWithMeasurements(Eigen::Vector3d f_bi_b,
                                                  Eigen::Vector3d omega_bi_b) {
  p_dot_n_ = D() * v_n_;
  v_dot_n_ = T_bn_ * f_bi_b + g_n_ - (Omega_ne_n() + 2 * Omega_ei_n()) * v_n_;

  Eigen::Matrix3d Omega_bi_b;
  Utils::toSkewSymmetricMatrix(Omega_bi_b, omega_bi_b);
  T_dot_bn_ = T_bn_ * Omega_bi_b - (Omega_ei_n() + Omega_ne_n()) * T_bn_;
}

void NavigationState::integrateState(double dt) {
  p_n_ += dt * p_dot_n_;
  v_n_ += dt * v_dot_n_;
  T_bn_ += dt * T_dot_bn_; // TODO: can be more precise with sin/cos method
}

Eigen::Matrix3d NavigationState::D() {
  Eigen::Matrix3d D_ = Eigen::Matrix3d::Zero();

  D_(0, 0) = 1 / (Utils::Rm(p_n_(phi)) + p_n_(h));
  D_(1, 1) = 1 / (std::cos(p_n_(phi)) * (Utils::Rn(p_n_(phi)) + p_n_(h)));
  D_(2, 2) = -1;

  return D_;
}

Eigen::Matrix3d NavigationState::Omega_ne_n() {
  Eigen::Vector3d omega_ne_n_;
  omega_ne_n_(0) = v_n_(e) / (Utils::Rn(p_n_(phi)) + p_n_(h));
  omega_ne_n_(1) = -v_n_(n) / (Utils::Rm(p_n_(phi)) + p_n_(h));
  omega_ne_n_(2) =
      -(v_n_(e) * std::tan(p_n_(phi))) / (Utils::Rn(p_n_(phi)) + p_n_(h));

  Eigen::Matrix3d Omega_ne_n_;
  Utils::toSkewSymmetricMatrix(Omega_ne_n_, omega_ne_n_);

  return Omega_ne_n_;
}

Eigen::Matrix3d NavigationState::Omega_ei_n() {
  Eigen::Vector3d omega_ei_n_;
  omega_ei_n_(0) = Utils::omega_ei * std::cos(p_n_(phi));
  omega_ei_n_(1) = 0;
  omega_ei_n_(2) = -Utils::omega_ei * std::sin(p_n_(phi));

  Eigen::Matrix3d Omega_ei_n_;
  Utils::toSkewSymmetricMatrix(Omega_ei_n_, omega_ei_n_);

  return Omega_ei_n_;
}
} // namespace EKF_INS
