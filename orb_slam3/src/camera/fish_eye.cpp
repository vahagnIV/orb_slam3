//
// Created by vahagn on 04/02/21.
//
// https://github.com/opencv/opencv/blob/master/modules/calib3d/src/fisheye.cpp

#include "camera/fish_eye.h"

namespace orb_slam3 {
namespace camera {

FishEye::FishEye(IDistortionModel<4>::EstimateType * estimate) : IDistortionModel(estimate) {

}

void FishEye::DistortPoint(const TPoint2D & undistorted, TPoint2D & distorted) {

  /*ACHTUNG: NOT TESTED*/

  const double & k1 = (*estimate_)[4];
  const double & k2 = (*estimate_)[5];
  const double & k3 = (*estimate_)[6];
  const double & k4 = (*estimate_)[7];

  double r2 = undistorted[0] * undistorted[0] + undistorted[1] * undistorted[1];
  double r = std::sqrt(r2);

  // Angle of the incoming ray:
  double theta = std::atan(r);

  double theta2 = theta * theta, theta3 = theta2 * theta, theta4 = theta2 * theta2, theta5 = theta4 * theta,
      theta6 = theta3 * theta3, theta7 = theta6 * theta, theta8 = theta4 * theta4, theta9 = theta8 * theta;

  double theta_d = theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;

  double inv_r = r > 1e-8 ? 1.0 / r : 1;
  double cdist = r > 1e-8 ? theta_d * inv_r : 1;

  distorted = undistorted * cdist;
}

void FishEye::UnDistortPoint(const TPoint2D & distorted, TPoint2D & undistorted) {

  const double & k1 = (*estimate_)[4];
  const double & k2 = (*estimate_)[5];
  const double & k3 = (*estimate_)[6];
  const double & k4 = (*estimate_)[7];

  double theta_d = sqrt(distorted[0] * distorted[0] + distorted[1] * distorted[1]);
  theta_d = std::min(std::max(-M_PI_2, theta_d), M_PI_2);
  bool converged = false;
  double theta = theta_d;

  double scale = 0.0;
  if (std::abs(theta_d) > 1e-8) {
    // compensate distortion iteratively

    const double EPS = 1e-8; // or std::numeric_limits<double>::epsilon();

    for (int j = 0; j < 10; j++) {
      double theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 = theta6 * theta2;
      double k0_theta2 = k1 * theta2, k1_theta4 = k2 * theta4, k2_theta6 = k3 * theta6, k3_theta8 = k4 * theta8;
      /* new_theta = theta - theta_fix, theta_fix = f0(theta) / f0'(theta) */
      double theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
          (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
      theta = theta - theta_fix;
      if (fabs(theta_fix) < EPS) {
        converged = true;
        break;
      }
    }

    scale = std::tan(theta) / theta_d;
  } else {
    converged = true;
  }

  bool theta_flipped = ((theta_d < 0 && theta > 0) || (theta_d > 0 && theta < 0));

  if (converged && !theta_flipped) {
    undistorted = distorted * scale; //undistorted point

  } else {
    undistorted << std::numeric_limits<double>::min(), std::numeric_limits<double>::min();
  }

}

}
}