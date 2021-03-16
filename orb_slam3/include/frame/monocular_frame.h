//
// Created by vahagn on 1/23/21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_MONOCULAR_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_MONOCULAR_FRAME_H_
// == dbow2 ===
#include <DBoW2/BowVector.h>
#include <DBoW2/FeatureVector.h>

// == orb-slam3 ===
#include <frame/frame_base.h>
#include <frame/frame_link.h>
#include <camera/monocular_camera.h>
#include <features/bow_vocabulary.h>

namespace orb_slam3 {
namespace frame {

class MonocularFrame : public FrameBase {
 public:
  MonocularFrame(const TImageGray8U &image,
                 TimePoint timestamp,
                 const std::shared_ptr<features::IFeatureExtractor> &feature_extractor,
                 const std::shared_ptr<camera::MonocularCamera> &camera,
                 features::BowVocabulary *vocabulary);

  // ==== FrameBase =========
  bool IsValid() const override;
  FrameType Type() const override;
  bool Link(const std::shared_ptr<FrameBase> &other) override;
  void AppendDescriptorsToList(size_t feature_id,
                               std::vector<features::DescriptorType> &out_descriptor_ptr) const override;
  TPoint3D GetNormal(const TPoint3D &point) const override;
  const camera::ICamera *CameraPtr() const override;
  bool TrackWithReferenceKeyFrame(const std::shared_ptr<FrameBase> &reference_keyframe) override;
  const features::Features &GetFeatures() const { return features_; }

  // ==== Monocular ====
  const FrameLink &GetFrameLink() const { return frame_link_; }
  void AppendToOptimizerBA(g2o::SparseOptimizer &optimizer, size_t &next_id) override;
  void CollectFromOptimizerBA(g2o::SparseOptimizer &optimizer) override;
  void OptimizePose(std::unordered_set<std::size_t> & out_inliers);
 protected:
  void ComputeBow();


 protected:
  features::Features features_;
  const std::shared_ptr<camera::MonocularCamera> camera_;
  features::BowVocabulary *vocabulary_;
  FrameLink frame_link_;
  DBoW2::BowVector bow_vector_;
  DBoW2::FeatureVector feature_vector_;
};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_MONOCULAR_FRAME_H_
