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
  MonocularFrame(const TImageGray8U & image,
                 TimePoint timestamp,
                 const std::shared_ptr<features::IFeatureExtractor> & feature_extractor,
                 const std::string & filename,
                 const std::shared_ptr<camera::MonocularCamera> & camera,
                 features::BowVocabulary *vocabulary);

  // ==== FrameBase =========
  bool IsValid() const override;
  FrameType Type() const override { return MONOCULAR; }
  bool Link(const std::shared_ptr<FrameBase> & other) override;
  void AppendDescriptorsToList(size_t feature_id,
                               std::vector<features::DescriptorType> & out_descriptor_ptr) const override;
  TPoint3D GetNormal(const TVector3D & point) const override;
  bool TrackWithReferenceKeyFrame(const std::shared_ptr<FrameBase> & reference_keyframe) override;
  const features::Features & GetFeatures() const { return features_; }

  // ==== Monocular ====
  const FrameLink & GetFrameLink() const { return frame_link_; }
  void AppendToOptimizerBA(g2o::SparseOptimizer & optimizer, size_t & next_id) override;
  void CollectFromOptimizerBA(g2o::SparseOptimizer & optimizer) override;
  void OptimizePose(std::unordered_set<std::size_t> & out_inliers);
  void FindNewMapPoints() override;
  precision_t ComputeMedianDepth() const override;
 protected:
  void ComputeBow();
  bool BaselineIsNotEnough(const MonocularFrame *other) const;
  void ComputeMatches(const MonocularFrame *other,
                      std::vector<features::Match> & out_matches,
                      geometry::Pose & out_pose) const;
 protected:
  features::Features features_;
  const std::shared_ptr<camera::MonocularCamera> camera_;
  FrameLink frame_link_;

};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_MONOCULAR_FRAME_H_
