//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_PINHOLE_CAMERA_H_
#define ORB_SLAM3_INCLUDE_PINHOLE_CAMERA_H_

#include "icamera.h"

namespace orb_slam3 {

class RGBDCamera : public ICamera {
 public:
  RGBDCamera(const TIntrinsicMatrix & rgb_intrinsic_matrix,
             const TDistortionCoefficients & rgb_distortion_coeffs,
             const T3DTransformationMatrix & rgbd2depth,
             int width,
             int height);

  void UndistortKeyPoints(const cv::Mat & points, cv::Mat & out_undistorted_points) override;

  const TIntrinsicMatrix & GetIntrinsicMatrix() const { return rgb_intrinsic_matrix_; }

  inline const float & Fx() const { return rgb_intrinsic_matrix_.val[0]; }
  inline const float & Fy() const { return rgb_intrinsic_matrix_.val[4]; }
  inline const float & Cx() const { return rgb_intrinsic_matrix_.val[2]; }
  inline const float & Cy() const { return rgb_intrinsic_matrix_.val[5]; }

 private:
  void ComputeImageBounds();

 private:
  float min_x_;
  float max_x_;
  float min_y_;
  float max_y_;

  TIntrinsicMatrix rgb_intrinsic_matrix_;
  TDistortionCoefficients rgb_distortion_coeffs_;
  T3DTransformationMatrix rgbd2depth_;
  int width_;
  int height_;

};

}

#endif //ORB_SLAM3_INCLUDE_PINHOLE_CAMERA_H_
