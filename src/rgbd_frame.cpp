//
// Created by vahagn on 11/30/20.
//


#include "rgbd_frame.h"

namespace nvision {

RGBDFrame::RGBDFrame(const ImageRGB8U & image, const ImageGray32F & depth, double timestamp,
                     const std::shared_ptr<RGBDCamera> & camera,
                     const std::shared_ptr<IFeatureExtractor> & feature_extractor) :
    FrameBase(timestamp, feature_extractor),
    camera_(camera) {

  if (image.empty())
    return;

  if (image.type() != CV_8UC3)
    return;

  cv::cvtColor(image, gray_image_, cv::COLOR_BGR2RGB);
}

int RGBDFrame::Compute() {

  int result;
  result = feature_extractor_->Extract(gray_image_, key_points_, descriptors_);
  if(0 != result)
    return result;

  camera_->UndistortKeyPoints(key_points_);

  return 0;
}

}