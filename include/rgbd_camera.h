//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_PINHOLE_CAMERA_H_
#define ORB_SLAM3_INCLUDE_PINHOLE_CAMERA_H_

#include <icamera.h>

namespace nvision {

class RGBDCamera : public ICamera {
 public:
  RGBDCamera(const TIntrinsicMatrix & rgb_intrinsic_matrix,
             const TDistortionCoefficients & rgb_distortion_coeffs,
             const T3DTransformationMatrix & rgbd2depth);

  void UndistortKeyPoints(std::vector<KeyPoint> & in_out_keypoints) override;

  const TIntrinsicMatrix & GetIntrinsicMatrix() const { return rgb_intrinsic_matrix_; }

 private:
  TIntrinsicMatrix rgb_intrinsic_matrix_;
  TDistortionCoefficients rgb_distortion_coeffs_;
  T3DTransformationMatrix rgbd2depth_;

};

}

#endif //ORB_SLAM3_INCLUDE_PINHOLE_CAMERA_H_
