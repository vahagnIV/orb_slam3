//
// Created by vahagn on 11/30/20.
//


#include "rgbd_frame.h"

namespace orb_slam3 {

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
    map_points_[i].xi.x = points.at<float>(i, 0);
    map_points_[i].xi.y = points.at<float>(i, 1);

    map_points_[i].x.x = (map_points_[i].xi.x - camera_->Cx()) / camera_->Fx();
    map_points_[i].x.y = (map_points_[i].xi.y - camera_->Cy()) / camera_->Fy();
    map_points_[i].x.z = depth_image_.at<float>(points.at<float>(i, 0), points.at<float>(i, 1));
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