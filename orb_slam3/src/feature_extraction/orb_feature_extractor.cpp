//
// Created by vahagn on 11/30/20.
//

#include "feature_extraction/orb_feature_extractor.h"

namespace orb_slam3 {
namespace feature_extraction {
ORBFeatureExtractor::ORBFeatureExtractor(unsigned image_width, unsigned image_height) : image_width_(image_width), image_height_(image_height) {

}

/*
 * For the time being we use opencv's implementation of orb feature extraction.
 * If, for some reason this doesn't work we will replace it with the original version
 * */
int ORBFeatureExtractor::Extract(const ImageGray8U & image,
                                 std::vector<KeyPoint> & out_keypoints,
                                 DescriptorSet & out_descriptors) {

  extractor_->detectAndCompute(image, cv::Mat(), out_keypoints, out_descriptors);

  return out_keypoints.size();
}

}
}