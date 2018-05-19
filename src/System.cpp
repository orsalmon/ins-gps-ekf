//
// Created by Or Salmon on 18/05/18.
//

#include "System.h"

namespace EKF {
System::System() : system_ptr_(this) {
  tracking_ptr_.reset(new EKF::Tracking());
}

} // namespace EKF