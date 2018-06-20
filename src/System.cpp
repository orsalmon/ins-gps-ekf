//
// Created by Or Salmon on 18/05/18.
//

#include "System.h"

namespace EKF_INS {
System::System() : system_ptr_(this) {
  tracking_ptr_.reset(new EKF_INS::Tracking());
}

} // namespace EKF_INS