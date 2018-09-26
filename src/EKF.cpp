//
// Created by Or Salmon on 22/06/18.
//

#include "EKF.h"

namespace EKF_INS {
EKF::EKF() : Q_(15, 15), R_(6, 6), H_(6, 15), is_running_(false) {
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
  tracker_ = new EKF_INS::Tracking();

  // Initializing default Q matrix
  Eigen::VectorXd q_diag(15);
  q_diag << 0.01, 0.01, 0.01, 0.5, 0.5, 0.1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3;
  Q_ = q_diag.asDiagonal();
  tracker_->setQMatrix(Q_);

  // Initializing default R matrix
  Eigen::VectorXd r_diag(6);
  r_diag << 0.000001, 0.000001, 0.5, 0.000001, 0.000001, 0.5;
  R_ = r_diag.asDiagonal();

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
  logger_->info("EKF::updateWithInertialMeasurement - p_n: {} v_n: {} T_n: {}",
                std::get<0>(ins_navigation_state_).transpose(),
                std::get<1>(ins_navigation_state_).transpose(),
                EKF_INS::Utils::toEulerAngles(std::get<2>(ins_navigation_state_)).transpose());
  // Set the most update state to provide
  state_mutex_.lock();
  current_navigation_state_ = ins_navigation_state_;
  current_error_state_ = ins_error_state_;
  current_state_covariance_ = ins_error_state_covariance_;
  state_mutex_.unlock();
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
  logger_->debug("EKF::updateWithGPSMeasurements - gps_data_mean: {}", gps_data_mean.transpose());
  Eigen::VectorXd pos_vel_state(6);
  pos_vel_state << std::get<0>(ins_navigation_state_), std::get<1>(ins_navigation_state_);
  logger_->debug("EKF::updateWithGPSMeasurements - pos_vel_state: {}", pos_vel_state.transpose());

  // Calculate measurement vector
  Eigen::VectorXd z = pos_vel_state - gps_data_mean;
  logger_->debug("EKF::updateWithGPSMeasurements - z: {}", z.transpose());

  // Calculate Kalman gain
  Eigen::MatrixXd P = ins_error_state_covariance_;
  logger_->debug("EKF::updateWithGPSMeasurements - ins_error_state_covariance:\n{}", ins_error_state_covariance_);
  Eigen::MatrixXd S = H_ * P * H_.transpose() + R_;
  Eigen::MatrixXd K = P * H_.transpose() * S.inverse();
  logger_->debug("EKF::updateWithGPSMeasurements - K:\n{}", K);

  // Correction step
  fixed_error_state_ = ins_error_state_ + K * (z - H_ * ins_error_state_);
  logger_->debug("EKF::updateWithGPSMeasurements - fixed_error_state: {}", fixed_error_state_.transpose());
  fixed_error_state_covariance_ = (Eigen::Matrix<double, 15, 15>::Identity() - K * H_) * P;
  // Position correction
  std::get<0>(fixed_navigation_state_) = std::get<0>(ins_navigation_state_) - fixed_error_state_.segment<3>(0);
  logger_->debug("EKF::updateWithGPSMeasurements - std::get<0>(fixed_navigation_state_): {}", std::get<0>(fixed_navigation_state_).transpose());
  // Velocity correction
  std::get<1>(fixed_navigation_state_) = std::get<1>(ins_navigation_state_) - fixed_error_state_.segment<3>(3);
  logger_->debug("EKF::updateWithGPSMeasurements - std::get<1>(fixed_navigation_state_): {}", std::get<1>(fixed_navigation_state_).transpose());
  // Orientation correction
  logger_->debug("EKF::updateWithGPSMeasurements - std::get<2>(fixed_navigation_state_): {}",
                 EKF_INS::Utils::toEulerAngles(std::get<2>(fixed_navigation_state_)).transpose());
  Eigen::Vector3d epsilon_n = fixed_error_state_.segment<3>(6);
  Eigen::Matrix3d E_n;
  Utils::toSkewSymmetricMatrix(E_n, epsilon_n);
  std::get<2>(fixed_navigation_state_) = (Eigen::Matrix3d::Identity() - E_n) * std::get<2>(ins_navigation_state_);

  logger_->debug("updateWithGPSMeasurements - error_state: \n{}", fixed_error_state_.transpose());

  // Reset step
  tracker_->error_state_ptr_->resetErrorState();
  tracker_->navigation_state_ptr_->setState(std::get<0>(fixed_navigation_state_),
                                            std::get<1>(fixed_navigation_state_),
                                            std::get<2>(fixed_navigation_state_));
  tracker_->error_state_covariance_ptr_->resetPMatrix();
  logger_->info("updateWithGPSMeasurements - p_n: {} v_n: {} T_n: {}",
                std::get<0>(fixed_navigation_state_).transpose(),
                std::get<1>(fixed_navigation_state_).transpose(),
                EKF_INS::Utils::toEulerAngles(std::get<2>(fixed_navigation_state_)).transpose());
  // Set the most update state to provide
  state_mutex_.lock();
  current_navigation_state_ = fixed_navigation_state_;
  current_error_state_ = fixed_error_state_;
  current_state_covariance_ = fixed_error_state_covariance_;
  state_mutex_.unlock();
}

Eigen::VectorXd EKF::getErrorState() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_error_state_;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> EKF::getNavigationState() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_navigation_state_;
}

Eigen::MatrixXd EKF::getErrorStateCovariance() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_state_covariance_;
}

Eigen::Vector3d EKF::getPositionState() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return std::get<0>(current_navigation_state_);
}

Eigen::Vector3d EKF::getVelocityState() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return std::get<1>(current_navigation_state_);
}

Eigen::Matrix3d EKF::getOrientationState() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return std::get<2>(current_navigation_state_);
}

void EKF::setQMatrix(Eigen::MatrixXd Q) {
  Q_ = Q;
  tracker_->setQMatrix(Q_);
}

void EKF::setRMatrix(Eigen::MatrixXd R) { R_ = R; }

void EKF::setInitialState(Eigen::Vector3d p_0, Eigen::Vector3d v_0, Eigen::Matrix3d T_0) {
  tracker_->setNavigationInitialState(p_0, v_0, T_0);
  state_mutex_.lock();
  current_navigation_state_ = std::make_tuple(p_0, v_0, T_0);
  state_mutex_.unlock();
}

void EKF::start() {
  tracker_->resetClock();
  is_running_ = true;
}
} // namespace EKF_INS