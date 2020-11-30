//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_PINHOLE_CAMERA_H_
#define ORB_SLAM3_INCLUDE_PINHOLE_CAMERA_H_

#include <icamera.h>

namespace nvision {

class PinholeCamera : public ICamera {
 public:
  void UndistortKeyPoints(std::vector<KeyPoint> &in_out_keypoints) override;
 private:
  IntrinsicMatrix camera_matrix_;
  DistortionCoefficients distortion_coefficients_;

};

}

#endif //ORB_SLAM3_INCLUDE_PINHOLE_CAMERA_H_
