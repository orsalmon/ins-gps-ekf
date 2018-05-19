//
// Created by Or Salmon on 18/05/18.
//

#ifndef EKF_SYSTEM_H
#define EKF_SYSTEM_H

#include "Core.h"
#include "Tracking.h"

namespace EKF {
class System {
 public:
  System();
  void setInitialState(Eigen::Vector3d p_0, Eigen::Vector3d v_0, Eigen::Matrix3d T_0);
  void start();

 private:
  std::shared_ptr<EKF::System> system_ptr_;
  std::shared_ptr<EKF::Tracking> tracking_ptr_;
};
} // namespace EKF

#endif // EKF_SYSTEM_H
