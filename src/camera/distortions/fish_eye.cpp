//
// Created by vahagn on 04/02/21.
//
// https://github.com/opencv/opencv/blob/master/modules/calib3d/src/fisheye.cpp

// == orb-slam3 ===
#include "fish_eye.h"

namespace orb_slam3 {
namespace camera {

FishEye::FishEye() : k1_(0), k2_(0), k3_(0), k4_(0) {
}

FishEye::FishEye(std::istream &istream, serialization::SerializationContext &context) {
  READ_FROM_STREAM(k1_, istream);
  READ_FROM_STREAM(k2_, istream);
  READ_FROM_STREAM(k3_, istream);
  READ_FROM_STREAM(k4_, istream);
}

DistortionModelType FishEye::Type() {
  return DistortionModelType::FISHEYE;
}

bool FishEye::DistortPoint(const HomogenousPoint &undistorted, HomogenousPoint &distorted) const {

  /*ACHTUNG: NOT TESTED*/

  double r2 = undistorted.x() * undistorted.x() + undistorted.y() * undistorted.y();
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

  distorted << undistorted.x() * cdist, undistorted.y() * cdist, 1;
  return true;
}

bool FishEye::UnDistortPoint(const HomogenousPoint & distorted, HomogenousPoint & undistorted) const {
//  std::vector<cv::Point2d> point(1), undistorted_point;
//  point[0] = cv::Point2d(distorted.x(), distorted.y());
//
//  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
//  cv::Mat K = R.clone();
//  cv::fisheye::undistortPoints(point,
//                               undistorted_point,
//                               K,
//                               cv::Mat(1, 4, CV_64F, (void *) (estimate_->data() + 4)),
//                               R,
//                               K);
//  undistorted << undistorted_point[0].x, undistorted_point[0].y, 1;
//  return true;

  double theta_d = std::sqrt(distorted.x() * distorted.x() + distorted.y() * distorted.y());
  theta_d = std::min(M_PI_2, std::max(-M_PI_2, theta_d));

  double theta = theta_d; // Initial guess
  static const int MAX_ITER = 10;
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
    double fp = 1 + 3 * K1() * theta_g2 + 5 * K2() * theta_g4 + 7 * K3() * theta_g6 + 9 * K4() * theta_g8;
    double theta_fix = (theta + K1() * theta_g3 + K2() * theta_g5 + K3() * theta_g7 + K4() * theta_g9 - theta_d) / fp;

    theta -= theta_fix;
    converged = std::abs(theta_fix) < 7e-8;
    if (converged) {
      break;
    }
  }
  if (!converged) {
    undistorted << -10000, -10000, -1;
    return false;
  }

  double scale = std::tan(theta) / theta_d;

  bool flipped = ((theta_d < 0 && theta > 0) || (theta_d > 0 && theta < 0));

  if (converged && !flipped && scale > 0) {
    undistorted.x() = distorted.x() * scale;
    undistorted.y() = distorted.y() * scale;
    undistorted.z() = 1;
    return true;
  }
  undistorted << -10000, -10000, -1;
  return false;
}

void FishEye::ComputeJacobian(const TPoint2D & point,
                              IDistortionModel::JacobianType & out_jacobian) const {
  const double & x = point[0];
  const double & y = point[1];
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
//  if(out_jacobian.array().isNaN().any())
//    throw std::runtime_error("Nan in jacobian");
}

void FishEye::Serialize(std::ostream & ostream) const {
  WRITE_TO_STREAM(k1_, ostream);
  WRITE_TO_STREAM(k2_, ostream);
  WRITE_TO_STREAM(k3_, ostream);
  WRITE_TO_STREAM(k4_, ostream);
}

}
}