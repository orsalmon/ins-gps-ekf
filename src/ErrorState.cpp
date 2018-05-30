//
// Created by Or Salmon on 18/05/18.
//

#include "ErrorState.h"

namespace EKF {
ErrorState::ErrorState(
    std::shared_ptr<EKF::NavigationState> navigation_state_ptr)
    : navigation_state_ptr_(navigation_state_ptr), gen_(rd_()),
      dis_(-1.0, 1.0)  {}

void ErrorState::getNavigationState() {
  auto fullState = navigation_state_ptr_->getState();
  p_n_ = std::get<0>(fullState);
  v_n_ = std::get<1>(fullState);
  T_bn_ = std::get<2>(fullState);
}

void ErrorState::updateStateWithMeasurements(Eigen::Vector3d f_bi_b,
                                             Eigen::Vector3d omega_bi_b) {
  getNavigationState();
  f_bi_b_ = f_bi_b;
  omega_bi_b_ = omega_bi_b;

  delta_p_dot_n_ = Frr() * delta_p_n_ + Frv() * delta_v_n_ + Fre() * epsilon_n_;
  delta_v_dot_n_ = Fvr() * delta_p_n_ + Fvv() * delta_v_n_ +
                   Fve() * epsilon_n_ + T_bn_ * b_a_ + T_bn_ * omega_a();
  epsilon_dot_n_ = Fer() * delta_p_n_ + Fev() * delta_v_n_ +
                   Fee() * epsilon_n_ + T_bn_ * b_g_ + T_bn_ * omega_g();
  b_dot_a_ = Fa() * b_a_ + omega_a_gm();
  b_dot_g_ = Fg() * b_g_ + omega_g_gm();
}

Eigen::Matrix3d ErrorState::Frr() {
  Eigen::Matrix3d Frr_ = Eigen::Matrix3d::Zero();
  Frr_(0, 2) = -v_n_(n) / std::pow(Utils::Rm(p_n_(phi)) + p_n_(h), 2);
  Frr_(1, 0) =
      v_n_(e) * std::sin(p_n_(phi)) /
      (std::pow(std::cos(p_n_(phi)), 2) * (Utils::Rn(p_n_(phi) + p_n_(h))));
  Frr_(1, 2) = -v_n_(e) / (std::cos(p_n_(phi)) *
                           std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2));

  return Frr_;
}

Eigen::Matrix3d ErrorState::Frv() {
  Eigen::Matrix3d Frv_ = Eigen::Matrix3d::Zero();
  Frv_(0, 0) = 1 / (Utils::Rm(p_n_(phi)) + p_n_(h));
  Frv_(1, 1) = 1 / (std::cos(p_n_(phi)) * (Utils::Rn(p_n_(phi)) + p_n_(h)));
  Frv_(2, 2) = -1;

  return Frv_;
}

Eigen::Matrix3d ErrorState::Fre() { return Eigen::Matrix3d::Zero(); }

Eigen::Matrix3d ErrorState::Fvr() {
  Eigen::Matrix3d Fvr_ = Eigen::Matrix3d::Zero();
  Fvr_(0, 0) = -2 * v_n_(e) * Utils::omega_ei * std::cos(p_n_(phi)) -
               std::pow(v_n_(e), 2) / ((Utils::Rn(p_n_(phi)) + p_n_(h)) *
                                       std::pow(std::cos(p_n_(phi)), 2));
  Fvr_(0, 2) =
      -v_n_(d) * v_n_(n) / std::pow(Utils::Rm(p_n_(phi)) + p_n_(h), 2) +
      std::pow(v_n_(e), 2) * std::tan(p_n_(phi)) /
          std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2);
  Fvr_(1, 0) =
      2 * Utils::omega_ei *
          (-v_n_(d) * std::sin(p_n_(phi)) + v_n_(n) * std::cos(p_n_(phi))) +
      v_n_(n) * v_n_(e) /
          ((Utils::Rn(p_n_(phi)) + p_n_(h)) * std::pow(std::cos(p_n_(phi)), 2));
  Fvr_(1, 2) =
      -v_n_(d) * v_n_(e) / std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2) -
      v_n_(n) * v_n_(e) * std::tan(p_n_(phi)) /
          std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2);
  Fvr_(2, 0) = 2 * v_n_(e) * Utils::omega_ei * std::sin(p_n_(phi));
  Fvr_(2, 2) =
      std::pow(v_n_(n), 2) / std::pow(Utils::Rm(p_n_(phi)) + p_n_(h), 2) +
      std::pow(v_n_(e), 2) / std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2) -
      2 * Utils::g / (Utils::Re + p_n_(h));

  return Fvr_;
}

Eigen::Matrix3d ErrorState::Fvv() {
  Eigen::Matrix3d Fvv_ = Eigen::Matrix3d::Zero();
  Fvv_(0, 0) = v_n_(d) / (Utils::Rm(p_n_(phi)) + p_n_(h));
  Fvv_(0, 1) =
      -2 * v_n_(e) * std::tan(p_n_(phi)) / (Utils::Rn(p_n_(phi)) + p_n_(h)) -
      2 * Utils::omega_ei * std::sin(p_n_(phi));
  Fvv_(0, 2) = v_n_(n) / (Utils::Rm(p_n_(phi)) + p_n_(h));
  Fvv_(1, 0) = 2 * Utils::omega_ei * std::sin(p_n_(phi)) +
               v_n_(e) * std::tan(p_n_(phi)) / (Utils::Rn(p_n_(phi)) + p_n_(h));
  Fvv_(1, 1) = v_n_(d) / (Utils::Rn(p_n_(phi)) + p_n_(h)) +
               v_n_(n) * std::tan(p_n_(phi)) / (Utils::Rn(p_n_(phi)) + p_n_(h));
  Fvv_(1, 2) = 2 * Utils::omega_ei * std::cos(p_n_(phi)) +
               v_n_(e) * std::tan(p_n_(phi)) / (Utils::Rn(p_n_(phi)) + p_n_(h));
  Fvv_(2, 0) = -2 * v_n_(n) / (Utils::Rm(p_n_(phi)) + p_n_(h));
  Fvv_(2, 1) = -2 * Utils::omega_ei * std::cos(p_n_(phi)) -
               2 * v_n_(e) / (Utils::Rn(p_n_(phi)) + p_n_(h));

  return Fvv_;
}

Eigen::Matrix3d ErrorState::Fve() {
  Eigen::Vector3d f_ins_n = T_bn_ * f_bi_b_;
  Eigen::Matrix3d F_ins_n;
  Utils::toSkewSymmetricMatrix(F_ins_n, f_ins_n);

  return -1 * F_ins_n;
}

Eigen::Matrix3d ErrorState::Fer() {
  Eigen::Matrix3d Fer_ = Eigen::Matrix3d::Zero();
  Fer_(0, 0) = Utils::omega_ei * std::sin(p_n_(phi));
  Fer_(0, 2) = v_n_(e) / std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2);
  Fer_(1, 2) = -v_n_(n) / std::pow(Utils::Rm(p_n_(phi)) + p_n_(h), 2);
  Fer_(2, 0) = Utils::omega_ei * std::cos(p_n_(phi)) +
               v_n_(e) / ((Utils::Rn(p_n_(phi) + p_n_(h))) *
                          std::pow(std::cos(p_n_(phi)), 2));
  Fer_(2, 2) = -v_n_(e) * std::tan(p_n_(phi)) /
               std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2);

  return Fer_;
}

Eigen::Matrix3d ErrorState::Fev() {
  Eigen::Matrix3d Fev_ = Eigen::Matrix3d::Zero();
  Fev_(0, 1) = -1 / (Utils::Rn(p_n_(phi)) + p_n_(h));
  Fev_(1, 0) = 1 / (Utils::Rm(p_n_(phi)) + p_n_(h));
  Fev_(2, 1) = std::tan(p_n_(phi)) / (Utils::Rn(p_n_(phi)) + p_n_(h));

  return Fev_;
}

Eigen::Matrix3d ErrorState::Fee() {
  Eigen::Vector3d omega_ins_n = T_bn_ * omega_bi_b_;
  Eigen::Matrix3d Omega_ins_n;
  Utils::toSkewSymmetricMatrix(Omega_ins_n, omega_ins_n);

  return -1 * Omega_ins_n;
}

Eigen::Matrix3d ErrorState::Fa() { return Eigen::Matrix3d::Zero(); }

Eigen::Matrix3d ErrorState::Fg() { return Eigen::Matrix3d::Zero(); }

Eigen::Vector3d ErrorState::omega_a() {
  Eigen::Vector3d omega_a_(dis_(gen_), dis_(gen_), dis_(gen_));

  return omega_a_max_ * omega_a_;
}

Eigen::Vector3d ErrorState::omega_g() {
  Eigen::Vector3d omega_g_(dis_(gen_), dis_(gen_), dis_(gen_));

  return omega_g_max_ * omega_g_;
}

Eigen::Vector3d ErrorState::omega_a_gm() {
  Eigen::Vector3d omega_a_gm_(dis_(gen_), dis_(gen_), dis_(gen_));

  return omega_a_gm_max_ * omega_a_gm_;
}

Eigen::Vector3d ErrorState::omega_g_gm() {
  Eigen::Vector3d omega_g_gm_(dis_(gen_), dis_(gen_), dis_(gen_));

  return omega_g_gm_max_ * omega_g_gm_;
}
} // namespace EKF
