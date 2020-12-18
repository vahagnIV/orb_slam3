//
// Created by vahagn on 11/29/20.
//

#include "frame_base.h"

namespace nvision {

void FrameBase::InitializeIdentity() noexcept {
  current_pose_ = cv::Matx44f::eye();
}

void FrameBase::CvKeypointsToMat(const std::vector<cv::KeyPoint> & keypoints, cv::Mat & out_points) {
  out_points.create(keypoints.size(), 2, CV_32F);
  for (int i = 0; i < keypoints.size(); ++i) {
    out_points.at<float>(i, 0) = keypoints[i].pt.x;
    out_points.at<float>(i, 1) = keypoints[i].pt.y;
  }
}

}