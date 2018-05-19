//
// Created by Or Salmon on 18/05/18.
//

#ifndef EKF_TRACKING_H
#define EKF_TRACKING_H

#include "Core.h"
#include "NavigationState.h"
#include "ErrorState.h"

namespace EKF {
class Tracking {
 public:
  Tracking();

 private:
  std::shared_ptr<EKF::NavigationState> navigation_state_ptr_;
  std::shared_ptr<EKF::ErrorState> error_state_ptr_;
};
} // namespace EKF

#endif // EKF_TRACKING_H
