//
// Created by Or Salmon on 22/06/18.
//

#include "EKF.h"

namespace EKF_INS {
EKF::EKF() : tracker_(new EKF_INS::Tracking()), Q_(15, 15), R_(6, 6), H_(6, 15), is_running_(false) {
  try {
    console_sink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink_->set_level(spdlog::level::info);
    console_sink_->set_pattern("[%^%l%$] %v");

    file_sink_ = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("ekf_ins.log", 1048576 * 5, 3);
    file_sink_->set_level(spdlog::level::trace);
    file_sink_->set_pattern("%+");

    logger_ = new spdlog::logger("ekf_ins_logger", {console_sink_, file_sink_});
    logger_->set_level(spdlog::level::debug);
    logger_->flush_on(spdlog::level::debug);

    logger_->info("Logging session has started");
  }
  catch (const spdlog::spdlog_ex &ex) {
    std::cout << "Log initialization failed: " << ex.what() << std::endl;
  }

  Q_.setZero();
  R_.setZero();
  H_.setZero();
  H_.topLeftCorner(6, 6).setIdentity();
}

void EKF::updateWithInertialMeasurement(Eigen::Vector3d data, EKF_INS::Type type) {
  if (!is_running_) {
    logger_->warn("EKF::updateWithInertialMeasurement - EKF not running yet");
    return;
  }
  switch (type) {
    case ACCELEROMETER:
      tracker_->updateTrackingWithAccelerometer(data);
      break;
    case GYRO:
      tracker_->updateTrackingWithGyro(data);
      break;
    default:
      logger_->error("EKF::updateWithInertialMeasurement - Can't accept this kind of measurement!");
  }
  ins_navigation_state_ = tracker_->getNavigationState();
  ins_error_state_ = tracker_->getErrorState();
  ins_error_state_covariance_ = tracker_->getErrorStateCovariance();
}

void EKF::updateWithGPSMeasurements(std::vector<Eigen::Matrix<double, 6, 1>> gps_data) {
  if (!is_running_) {
    logger_->warn("EKF::updateWithGPSMeasurements - EKF not running yet");
    return;
  }
  if (gps_data.empty()) {
    logger_->warn("EKF::updateWithGPSMeasurements - Got empty GPS data");
    return;
  }

  // Arrange gps data in error state manner
  auto gps_data_mean = Utils::calcMeanVector(gps_data);
  Eigen::VectorXd pos_vel_state(6);
  pos_vel_state << std::get<0>(ins_navigation_state_), std::get<1>(ins_navigation_state_);

  // Calculate measurement vector
  Eigen::VectorXd z = pos_vel_state - gps_data_mean;

  // Calculate Kalman gain
  Eigen::MatrixXd P = ins_error_state_covariance_;
  Eigen::MatrixXd K = P * H_.transpose() * (H_ * P * H_.transpose() + R_);

  // Correction step
  fixed_error_state_ = ins_error_state_ + K * (z - H_ * ins_error_state_);
  fixed_error_state_covariance_ =
      (Eigen::Matrix<double, 15, 15>::Identity() - K * H_) * P;
  // Position correction
  std::get<0>(fixed_navigation_state_) = std::get<0>(ins_navigation_state_) - fixed_error_state_.segment<3>(0);
  // Velocity correction
  std::get<1>(fixed_navigation_state_) = std::get<1>(ins_navigation_state_) - fixed_error_state_.segment<3>(3);
  // Orientation correction
  // TODO: complete

  // Reset step
  tracker_->error_state_ptr_->resetErrorState();
  tracker_->navigation_state_ptr_->setState(std::get<0>(fixed_navigation_state_),
                                            std::get<1>(fixed_navigation_state_),
                                            std::get<2>(fixed_navigation_state_));
}

Eigen::VectorXd EKF::getFixedErrorState() { return fixed_error_state_; }

auto EKF::getFixedNavigationState() {
  return fixed_navigation_state_;
}

Eigen::MatrixXd EKF::getFixedErrorStateCovariance() {
  return fixed_error_state_covariance_;
}

void EKF::setQMatrix(Eigen::MatrixXd Q) {
  Q_ = Q;
  tracker_->setQMatrix(Q_);
}

void EKF::setRMatrix(Eigen::MatrixXd R) { R_ = R; }

void EKF::setInitialState(Eigen::Vector3d p_0, Eigen::Vector3d v_0,
                          Eigen::Matrix3d T_0) {
  tracker_->setNavigationInitialState(p_0, v_0, T_0);
}

void EKF::start() {
  tracker_->resetClock();
  is_running_ = true;
}
} // namespace EKF_INS