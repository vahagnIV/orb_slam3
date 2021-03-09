//
// Created by vahagn on 04/02/21.
//
// https://github.com/opencv/opencv/blob/master/modules/calib3d/src/fisheye.cpp

// == orb-slam3 ===
#include "camera/fish_eye.h"

namespace orb_slam3 {
namespace camera {

FishEye::FishEye(IDistortionModel<4>::EstimateType *estimate) : IDistortionModel(estimate) {

}

bool FishEye::DistortPoint(const HomogenousPoint &undistorted, HomogenousPoint &distorted) {

  /*ACHTUNG: NOT TESTED*/

  double r2 = undistorted[0] * undistorted[0] + undistorted[1] * undistorted[1];
  double r = std::sqrt(r2);

  // Angle of the incoming ray:
  double theta = std::atan(r);

  double theta2 = theta * theta,
      theta3 = theta2 * theta,
      theta4 = theta2 * theta2,
      theta5 = theta4 * theta,
      theta6 = theta3 * theta3,
      theta7 = theta6 * theta,
      theta8 = theta4 * theta4,
      theta9 = theta8 * theta;

  double theta_d = theta + K1() * theta3 + K2() * theta5 + K3() * theta7 + K4() * theta9;

  double inv_r = r > 1e-8 ? 1.0 / r : 1;
  double cdist = r > 1e-8 ? theta_d * inv_r : 1;

  distorted << undistorted[0] * cdist, undistorted[1] * cdist, 1;
  return true;
}

bool FishEye::UnDistortPoint(const HomogenousPoint &distorted, HomogenousPoint &undistorted) {

  double r_prime = std::sqrt(distorted[0] * distorted[0] + distorted[1] * distorted[1]);

  double theta = std::atan(r_prime); // Initial guess
  static const int MAX_ITER = 10;
  double theta_d;
  bool converged = false;
  for (int i = 0; i < MAX_ITER; ++i) {
    double theta_g2 = theta * theta;
    double theta_g3 = theta_g2 * theta;
    double theta_g4 = theta_g3 * theta;
    double theta_g5 = theta_g4 * theta;
    double theta_g6 = theta_g5 * theta;
    double theta_g7 = theta_g6 * theta;
    double theta_g8 = theta_g7 * theta;
    double theta_g9 = theta_g8 * theta;
    theta_d = theta + K1() * theta_g3 + K2() * theta_g5 + K3() * theta_g7 + K4() * theta_g9;
    double fp = 1 + 3 * K1() * theta_g2 + 5 * K2() * theta_g4 + 7 * K3() * theta_g6 + 9 * K4() * theta_g8;
    // double fpp = 6 * K1() * theta + 20 * K2() * theta_g3 + 42 * K3() * theta_g5 + 72 * K4() * theta_g7;
    // double delta_theta = -theta_d * fp / (fp * fp + fpp * theta_d);

    double delta_theta = -(theta_d - r_prime) / fp;
    converged = std::abs(delta_theta) < 7e-8;
    if (converged) {
      break;
    }
    theta += delta_theta;
  }
  if (!converged) {
    undistorted = distorted;
    return false;
  }

  double scale = std::tan(theta) / theta_d;

  if (scale < 0) {
    undistorted = distorted;
    return false;
  }
  undistorted << distorted[0] * scale, distorted[1] * scale, 1;
  return true;
}

void FishEye::GetTransformationJacobian(const HomogenousPoint &point, IDistortionModel<4>::JacobianType &out_jacobian) {
  const double &x = point[0];
  const double &y = point[1];
  double x2 = x * x, y2 = y * y;
  double r2 = x2 + y2;
  double r = std::sqrt(r2);
  double r3 = r2 * r;
  double theta = std::atan2(r, 1);

  double theta2 = theta * theta, theta3 = theta2 * theta;
  double theta4 = theta2 * theta2, theta5 = theta4 * theta;
  double theta6 = theta2 * theta4, theta7 = theta6 * theta;
  double theta8 = theta4 * theta4, theta9 = theta8 * theta;

  double thetad = theta + theta3 * K1() + theta5 * K2() + theta7 * K3() +
      theta9 * K4();
  double Dthetad = 1 + 3 * K1() * theta2 + 5 * K2() * theta4 + 7 * K3() * theta6 +
      9 * K4() * theta8;

  out_jacobian(0, 0) = (Dthetad * x2 / (r2 * (r2 + 1)) + thetad * y2 / r3);

  out_jacobian(0, 1) = out_jacobian(1, 0) = (Dthetad * y * x / (r2 * (r2 + 1)) - thetad * y * x / r3);

//  out_jacobian(1, 0) = (Dthetad * y * x / (r2 * (r2 + 1)) - thetad * y * x / r3);
  out_jacobian(1, 1) = (Dthetad * y2 / (r2 * (r2 + 1)) + thetad * x2 / r3);
}

}
}