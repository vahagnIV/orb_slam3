//
// Created by vahagn on 1/23/21.
//

#include <frame/monocular_frame.h>

namespace orb_slam3 {
namespace frame {
MonocularFrame::MonocularFrame(const TImageGray8U & image,
                               TimePoint timestamp,
                               const std::shared_ptr<feature_extraction::IFeatureExtractor> & feature_extractor,
                               const std::shared_ptr<camera::MonocularCamera> & camera)
    : FrameBase(timestamp, feature_extractor), camera_(camera) {
  TKeyPoints key_points;
  DescriptorSet descriptors;
  feature_extractor_->Extract(image, key_points, descriptors);

}

}
}