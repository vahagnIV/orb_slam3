//
// Created by vahagn on 11/29/20.
//

#include "frame_base.h"

namespace nvision {

FrameBase::FrameBase() : reference_frame_(nullptr) {

}

void FrameBase::InitializeIdentity() noexcept {
  SetPosition(cv::Matx44f::eye());
}

void FrameBase::CvKeypointsToMat(const std::vector<cv::KeyPoint> &keypoints, cv::Mat &out_points) {
  out_points.create(keypoints.size(), 2, CV_32F);
  for (int i = 0; i < keypoints.size(); ++i) {
    out_points.at<float>(i, 0) = keypoints[i].pt.x;
    out_points.at<float>(i, 1) = keypoints[i].pt.y;
  }
}

void FrameBase::SetPosition(const cv::Matx44f &position) noexcept {
  for (int i = 0; i < 3; ++i) {
    T_camera2world_(i) = position(3, i);
    for (int j = 0; j < 3; ++j) {
      R_camera2world_(i, j) = position(i, j);
    }
  }
  R_world2camera_ = R_camera2world_.t();

  T_0world_ = -R_world2camera_ * T_camera2world_;
}

}