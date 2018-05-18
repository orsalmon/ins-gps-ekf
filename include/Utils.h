//
// Created by Or Salmon on 18/05/18.
//

#ifndef EKF_UTILS_H
#define EKF_UTILS_H

#include <Eigen/Eigen>
#include <math.h>

namespace EKF {
class Utils {
public:
  static inline double getMeridianRadius(double phi) {
    return (Re * (1 - e2) / std::pow(1 - e2 * std::pow(std::sin(phi), 2), 3 / 2));
  }

  static inline double getNormalRadius(double phi) {
    return (Re / std::pow(1 - e2 * std::pow(std::sin(phi), 2), 1 / 2));
  }

  static void toSkewSymmetricMatrix(Eigen::Matrix3d &M, Eigen::Vector3d &v) {
    M <<  0  , -v(2),  v(1),
         v(2),   0  , -v(0),
        -v(1),  v(0),   0  ;
  }

  static constexpr double Re = 6378.137E3; // Earth semi-major axis [m]
  static constexpr double e = 0.081819;    // Earth eccentricity
  static constexpr double e2 = std::pow(e, 2);
};
} // namespace EKF

#endif // EKF_UTILS_H
