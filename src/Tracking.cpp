//
// Created by or-caja on 18/05/18.
//

#include "Tracking.h"

namespace EKF {
Tracking::Tracking() {
  navigation_state_ptr_.reset(new EKF::NavigationState());
  error_state_ptr_.reset(new EKF::ErrorState(navigation_state_ptr_));
}
} // namespace EKF
