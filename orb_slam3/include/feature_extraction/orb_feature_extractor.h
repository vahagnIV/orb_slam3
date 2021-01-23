//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_ORB_FEATURE_EXTRACTOR_H_
#define ORB_SLAM3_INCLUDE_ORB_FEATURE_EXTRACTOR_H_

#include <feature_extraction/ifeature_extractor.h>

namespace orb_slam3 {
namespace feature_extraction {

class ORBFeatureExtractor : public IFeatureExtractor {
 public:
  ORBFeatureExtractor(unsigned image_width, unsigned image_height);
  int Extract(const TImageGray8U & image,
              TKeyPoints & out_keypoints,
              DescriptorSet & out_descriptors) override;
 private:
  unsigned image_width_;
  unsigned image_height_;

};

}
}
#endif //ORB_SLAM3_INCLUDE_ORB_FEATURE_EXTRACTOR_H_
