//
// Created by vahagn on 1/23/21.
//

#include <frame/monocular_frame.h>

namespace orb_slam3 {
namespace frame {
MonocularFrame::MonocularFrame(const ImageRGB8U & image,
                               TimePoint timestamp,
                               const std::shared_ptr<feature_extraction::IFeatureExtractor> & feature_extractor)
    : FrameBase(timestamp, feature_extractor) {

}

}
}