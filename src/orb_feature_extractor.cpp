//
// Created by vahagn on 11/30/20.
//

#include "orb_feature_extractor.h"

namespace orb_slam3 {

ORBFeatureExtractor::ORBFeatureExtractor() : extractor_(cv::ORB::create(1000)) {

}

/*
 * For the time being we use opencv's implementation of orb feature extraction.
 * If, for some reason this doesn't work we will replace it with the original version
 * */
int ORBFeatureExtractor::Extract(const ImageGray8U &image,
                                 std::vector<KeyPoint> &out_keypoints,
                                 DescriptorSet &out_descriptors) {

  extractor_->detectAndCompute(image, cv::Mat(), out_keypoints, out_descriptors);

  return out_keypoints.size();
}

}