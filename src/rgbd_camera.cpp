//
// Created by vahagn on 11/30/20.
//

#include "rgbd_camera.h"

namespace nvision {

RGBDCamera::RGBDCamera(const TIntrinsicMatrix & rgb_intrinsic_matrix,
                       const TDistortionCoefficients & rgb_distortion_coeffs,
                       const T3DTransformationMatrix & rgbd2depth,
                       int width,
                       int height) :
    rgb_intrinsic_matrix_(rgb_intrinsic_matrix),
    rgb_distortion_coeffs_(rgb_distortion_coeffs),
    rgbd2depth_(rgbd2depth),
    width_(width),
    height_(height) {
  ComputeImageBounds();
}

void RGBDCamera::UndistortKeyPoints(const cv::Mat & points, cv::Mat & out_undistorted_points) {
  if (0 == rgb_distortion_coeffs_.at<float>(0)) return;

//  // Fill matrix with points
//  cv::Mat mat(keypoints.size(), 2, CV_32F);
//
//  for (int i = 0; i < keypoints.size(); i++) {
//    mat.at<float>(i, 0) = keypoints[i].pt.x;
//    mat.at<float>(i, 1) = keypoints[i].pt.y;
//  }

  // Undistort points
  //mat = mat.reshape(2);
  // TODO: fix the last parameter
  cv::undistortPoints(points, out_undistorted_points, rgb_intrinsic_matrix_, rgb_distortion_coeffs_, cv::Mat(), rgbd2depth_);
  out_undistorted_points = out_undistorted_points.reshape(1);

  /*for (int i = 0; i < keypoints.size(); i++) {
    keypoints[i].pt.x = mat.at<float>(i, 0);
    keypoints[i].pt.y = mat.at<float>(i, 1);
  }*/

}

void RGBDCamera::ComputeImageBounds() {
  if (rgb_distortion_coeffs_.at<float>(0) != 0.0) {
    cv::Mat mat(4, 2, CV_32F);
    mat.at<float>(0, 0) = 0.0;
    mat.at<float>(0, 1) = 0.0;
    mat.at<float>(1, 0) = width_;
    mat.at<float>(1, 1) = 0.0;
    mat.at<float>(2, 0) = 0.0;
    mat.at<float>(2, 1) = height_;
    mat.at<float>(3, 0) = width_;
    mat.at<float>(3, 1) = height_;

    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, rgb_intrinsic_matrix_, rgb_distortion_coeffs_, cv::Mat(), rgbd2depth_);
    mat = mat.reshape(1);

    // Undistort corners
    min_x_ = std::min(mat.at<float>(0, 0), mat.at<float>(2, 0));
    max_x_ = std::max(mat.at<float>(1, 0), mat.at<float>(3, 0));
    min_y_ = std::min(mat.at<float>(0, 1), mat.at<float>(1, 1));
    max_y_ = std::max(mat.at<float>(2, 1), mat.at<float>(3, 1));
  } else {
    min_x_ = 0.0f;
    max_x_ = width_;
    min_y_ = 0.0f;
    max_y_ = height_;
  }
}

}