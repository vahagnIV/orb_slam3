//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_ORB_FEATURE_EXTRACTOR_H_
#define ORB_SLAM3_INCLUDE_ORB_FEATURE_EXTRACTOR_H_

#include <ifeature_extractor.h>

namespace nvision {

class ORBFeatureExtractor: public IFeatureExtractor {
 public:
  ORBFeatureExtractor();
  int Extract(const ImageGray8U &image, std::vector<KeyPoint> &out_keypoints, DescriptorSet &out_descriptors) override;
 private:
  cv::Ptr<cv::ORB > extractor_;


};

}

#endif //ORB_SLAM3_INCLUDE_ORB_FEATURE_EXTRACTOR_H_
