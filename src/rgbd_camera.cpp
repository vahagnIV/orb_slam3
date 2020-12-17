//
// Created by vahagn on 11/30/20.
//

#include "rgbd_camera.h"

namespace nvision {

RGBDCamera::RGBDCamera(const TIntrinsicMatrix & rgb_intrinsic_matrix,
                       const TDistortionCoefficients & rgb_distortion_coeffs,
                       const T3DTransformationMatrix & rgbd2depth) :
    rgb_intrinsic_matrix_(rgb_intrinsic_matrix),
    rgb_distortion_coeffs_(rgb_distortion_coeffs),
    rgbd2depth_(rgbd2depth) {

}

void RGBDCamera::UndistortKeyPoints(std::vector<KeyPoint> & in_out_keypoints) {
  if (0 == rgb_distortion_coeffs_.at<float>(0)) return;

  // Fill matrix with points
  cv::Mat mat(in_out_keypoints.size(), 2, CV_32F);

  for (int i = 0; i < in_out_keypoints.size(); i++) {
    mat.at<float>(i, 0) = in_out_keypoints[i].pt.x;
    mat.at<float>(i, 1) = in_out_keypoints[i].pt.y;
  }

  // Undistort points
  mat = mat.reshape(2);
  // TODO: fix the last parameter
  cv::undistortPoints(mat, mat, rgb_intrinsic_matrix_, rgb_distortion_coeffs_, cv::Mat(), rgbd2depth_);
  mat = mat.reshape(1);

  for (int i = 0; i < in_out_keypoints.size(); i++) {
    in_out_keypoints[i].pt.x = mat.at<float>(i, 0);
    in_out_keypoints[i].pt.y = mat.at<float>(i, 1);
  }

}
}