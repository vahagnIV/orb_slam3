//
// Created by vahagn on 11/30/20.
//


#include "rgbd_frame.h"

namespace nvision {

RGBDFrame::RGBDFrame(const ImageRGB8U & image, const ImageGray32F & depth, double timestamp,
                     const std::shared_ptr<RGBDCamera> & camera,
                     const std::shared_ptr<IFeatureExtractor> & feature_extractor) :
    FrameBase(timestamp, feature_extractor),
    gray_image_(image),
    depth_image_(depth),
    camera_(camera) {

  if (image.empty())
    return;

  if (image.type() != CV_8UC3)
    return;

  cv::cvtColor(image, gray_image_, cv::COLOR_BGR2RGB);
}

void RGBDFrame::InitializeMapPoints(const cv::Mat & points, const cv::Mat & undistorted_points) {
  map_points_.resize(points.rows);
  for (int i = 0; i < points.rows; ++i) {
    map_points_[i].pt.x = points.at<float>(i, 0);
    map_points_[i].pt.y = points.at<float>(i, 1);
    map_points_[i].depth = depth_image_.at<float>(points.at<float>(i, 0), points.at<float>(i, 1));
  }
}

int RGBDFrame::Compute() {

  int result;
  std::vector<cv::KeyPoint> key_points;
  result = feature_extractor_->Extract(gray_image_, key_points, descriptors_);
  if (0 == result)
    return result;

  cv::Mat points, undistorted_points;
  this->CvKeypointsToMat(key_points, points);

  camera_->UndistortKeyPoints(points, undistorted_points);
  InitializeMapPoints(points, undistorted_points);

  return 0;
}

void RGBDFrame::AssignFeaturesToGrid() {

}

}