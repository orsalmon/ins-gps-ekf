//
// Created by Or Salmon on 18/05/18.
//

#ifndef EKF_INS_UTILS_H
#define EKF_INS_UTILS_H

#include <Eigen/Eigen>
#include <math.h>
#include <iostream>

namespace EKF_INS {
class Utils {
 public:
  static inline double Rm(double phi) {
    return (Re * (1 - e2) /
        std::pow(1 - e2 * std::pow(std::sin(phi), 2), 3 / 2));
  }

  static inline double Rn(double phi) {
    return (Re / std::pow(1 - e2 * std::pow(std::sin(phi), 2), 1 / 2));
  }

  static void toSkewSymmetricMatrix(Eigen::Matrix3d &M, Eigen::Vector3d &v) {
    M << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  }

  static inline double constrainAngleDegree(double x) {
    x = std::fmod(x + 180.0, 360.0);
    if (x < 0.0)
      x += 360.0;
    return x - 180.0;
  }

  static inline double degreeToRadian(double x) {
    return constrainAngleDegree(x) * M_PI / 180.0;
  }

  static inline double radianToDegree(double x) {
    return x * 180.0 / M_PI;
  }

  static Eigen::Vector3d toEulerAngles(Eigen::Matrix3d T) {
    Eigen::Vector3d eulerAngles;
    if (std::abs(T(2, 0)) != 1) {
      eulerAngles(PITCH) = -std::asin(T(2, 0));
      eulerAngles(ROLL) = std::atan2(T(2, 1) / std::cos(eulerAngles(PITCH)), T(2, 2) / std::cos(eulerAngles(PITCH)));
      eulerAngles(YAW) = std::atan2(T(1, 0) / std::cos(eulerAngles(PITCH)), T(0, 0) / std::cos(eulerAngles(PITCH)));
    } else {
      eulerAngles(YAW) = 0;
      if (T(2, 0) == -1) {
        eulerAngles(PITCH) = M_PI_2;
        eulerAngles(ROLL) = std::atan2(T(0, 1), T(0, 2));
      } else {
        eulerAngles(PITCH) = -M_PI_2;
        eulerAngles(ROLL) = std::atan2(-T(0, 1), -T(0, 2));
      }
    }

    return eulerAngles;
  }

  template<class T>
  static T calcMeanVector(std::vector<T> measurments) {
    T mean;
    mean.setZero();
    for (int n = 0; n < measurments.size(); ++n) {
      for (int i = 0; i < mean.rows(); ++i) {
        mean(i) = mean(i) + 1 / (n + 1) * (measurments.at(n)(i) - mean(i));
      }
    }
    return mean;
  }

  static constexpr double Re = 6378.137E3; // Earth semi-major axis [m]
  static constexpr double e = 0.081819;    // Earth eccentricity
  static constexpr double e2 = std::pow(e, 2);
  static constexpr double omega_ei = 7.292E-05; // Earth turn rate [rad/s]
  static constexpr double g = 9.80665;          // Gravity of Earth [m/s^2]

  enum Euler { ROLL, PITCH, YAW };

  struct FullNavigationState {
    Eigen::Vector3d p_n;
    Eigen::Vector3d v_n;
    Eigen::Matrix3d T_bn;
  };
};
} // namespace EKF_INS

#endif // EKF_INS_UTILS_H
