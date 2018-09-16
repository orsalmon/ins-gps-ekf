//
// Created by Or Salmon on 22/06/18.
//

#ifndef EKF_INS_EKF_H
#define EKF_INS_EKF_H

#include "Core.h"
#include "Tracking.h"

namespace EKF_INS {
enum Type { ACCELEROMETER, GYRO };
class EKF {
 public:
  EKF();
  void updateWithInertialMeasurement(Eigen::Vector3d data, enum Type type);
  void updateWithGPSMeasurements(std::vector<Eigen::Matrix<double, 6, 1>> gps_data);
  Eigen::VectorXd getErrorState();
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> getNavigationState();
  Eigen::MatrixXd getErrorStateCovariance();
  void setQMatrix(Eigen::MatrixXd Q);
  void setRMatrix(Eigen::MatrixXd R);
  void setInitialState(Eigen::Vector3d p_0, Eigen::Vector3d v_0, Eigen::Matrix3d T_0);
  void start();

 private:
  Tracking *tracker_;

  Eigen::VectorXd fixed_error_state_, ins_error_state_, current_error_state_;
  Eigen::MatrixXd fixed_error_state_covariance_, ins_error_state_covariance_, current_state_covariance_;
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> ins_navigation_state_, fixed_navigation_state_, current_navigation_state_;

  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd H_;

  bool is_running_;

  spdlog::logger *logger_;
  std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> console_sink_;
  std::shared_ptr<spdlog::sinks::rotating_file_sink_mt> file_sink_;
};
} // namespace EKF_INS

#endif // EKF_INS_EKF_H
