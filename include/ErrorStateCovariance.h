//
// Created by Or Salmon on 20/06/18.
//

#ifndef EKF_INS_ERRORSTATECOVARIANCE_H
#define EKF_INS_ERRORSTATECOVARIANCE_H

#include "Core.h"
#include "ErrorState.h"

namespace EKF_INS {
class ErrorStateCovariance {
 public:
  ErrorStateCovariance(std::shared_ptr<EKF_INS::ErrorState> error_state_ptr);
  ErrorStateCovariance(std::shared_ptr<EKF_INS::ErrorState> error_state_ptr, Eigen::MatrixXd Q);
  void setQMatrix(Eigen::MatrixXd Q);
  void updateCovarianceMatrix(double dt);
  Eigen::MatrixXd getErrorStateCovariance();
  void resetPMatrix();

 private:
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd P_;

  std::shared_ptr<EKF_INS::ErrorState> error_state_ptr_;
};
} // namespace EKF_INS

#endif // EKF_INS_ERRORSTATECOVARIANCE_H
