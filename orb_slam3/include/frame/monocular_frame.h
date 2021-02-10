//
// Created by vahagn on 1/23/21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_MONOCULAR_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_MONOCULAR_FRAME_H_

#include "frame_base.h"
#include <camera/monocular_camera.h>

namespace orb_slam3 {
namespace frame {

class MonocularFrame : public FrameBase {
 public:
  MonocularFrame(const TImageGray8U & image,
                 TimePoint timestamp,
                 const std::shared_ptr<features::IFeatureExtractor> & feature_extractor,
                 const std::shared_ptr<camera::MonocularCamera> & camera);

  size_t FeatureCount() const noexcept override;
  bool IsValid() const override;
  FrameType Type() const override;
  bool InitializePositionFromPrevious() override;

 protected:
  features::Features features_;
  const std::shared_ptr<camera::MonocularCamera> camera_;
};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_MONOCULAR_FRAME_H_
