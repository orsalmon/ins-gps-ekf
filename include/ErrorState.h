//
// Created by Or Salmon on 18/05/18.
//

#ifndef EKF_INS_ERRORSTATE_H
#define EKF_INS_ERRORSTATE_H

#include "Core.h"
#include "NavigationState.h"
#include <random>

namespace EKF_INS {
class ErrorState {
public:
  ErrorState(std::shared_ptr<EKF_INS::NavigationState> navigation_state_ptr);
  Eigen::VectorXd getState();
  void updateStateWithMeasurements(Eigen::Vector3d f_bi_b,
                                   Eigen::Vector3d omega_bi_b);
  void integrateState(double dt);
  void resetErrorState();
  void setOmegaA(double omega_a_max) { omega_a_max_ = omega_a_max; }
  void setOmegaG(double omega_g_max) { omega_g_max_ = omega_g_max; }
  void setOmegaAgm(double omega_a_gm_max) { omega_a_gm_max_ = omega_a_gm_max; }
  void setOmegaGgm(double omega_g_gm_max) { omega_g_gm_max_ = omega_g_gm_max; }
  Eigen::MatrixXd getTransitionMatrix(double dt);

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
  Eigen::Vector3d omega_a();
  Eigen::Vector3d omega_g();
  Eigen::Vector3d omega_a_gm();
  Eigen::Vector3d omega_g_gm();
  Eigen::MatrixXd F();
  void getNavigationState();

  Eigen::Vector3d delta_p_dot_n_, delta_p_n_;
  Eigen::Vector3d delta_v_dot_n_, delta_v_n_;
  Eigen::Vector3d epsilon_dot_n_, epsilon_n_;
  Eigen::Vector3d b_dot_a_, b_a_;
  Eigen::Vector3d b_dot_g_, b_g_;
  Eigen::VectorXd delta_x_;
  Eigen::MatrixXd F_;

  Eigen::Vector3d p_n_, v_n_;
  Eigen::Matrix3d T_bn_;
  Eigen::Vector3d f_bi_b_, omega_bi_b_;
  std::shared_ptr<EKF_INS::NavigationState> navigation_state_ptr_;

  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> dis_;
  double omega_a_max_, omega_g_max_, omega_a_gm_max_, omega_g_gm_max_;

  enum Pose { phi, lambda, h };
  enum Vel { n, e, d };
};
} // namespace EKF_INS

#endif // EKF_INS_ERRORSTATE_H
