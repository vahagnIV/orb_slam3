//
// Created by vahagn on 1/23/21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_MONOCULAR_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_MONOCULAR_FRAME_H_

// == orb-slam3 ===
#include <frame/frame_base.h>
#include <camera/monocular_camera.h>
#include <frame/frame_link.h>

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
  bool Link(const std::shared_ptr<FrameBase> & other) override;
  void AppendDescriptorsToList(size_t feature_id,
                               std::vector<features::DescriptorType> & out_descriptor_ptr) const override;

 protected:
  features::Features features_;
  const std::shared_ptr<camera::MonocularCamera> camera_;
  FrameLink frame_link_;
};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_MONOCULAR_FRAME_H_
