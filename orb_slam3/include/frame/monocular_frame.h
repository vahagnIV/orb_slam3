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

namespace test{
class MonocularFrameTests;
}

class MonocularFrame : public FrameBase {
  friend class MonocularFrameTests;
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
  TVector3D GetNormal(const TVector3D & point) const override;
  bool TrackWithReferenceKeyFrame(const std::shared_ptr<FrameBase> & reference_keyframe) override;
  bool TrackLocalMap(const std::shared_ptr<frame::FrameBase> & last_keyframe) override;
  const features::Features & GetFeatures() const { return features_; }


  // ==== Monocular ====
  void AppendToOptimizerBA(g2o::SparseOptimizer & optimizer, size_t & next_id) override;
  void CollectFromOptimizerBA(g2o::SparseOptimizer & optimizer) override;
  void OptimizePose(std::unordered_set<std::size_t> & out_inliers);
  bool FindNewMapPoints() override;
  precision_t ComputeMedianDepth() const override;
 private:
  typedef struct {
    size_t to_idx;
    size_t from_idx;
    size_t edge_id;
    MonocularFrame * frame;
  } MapPointMatch;

  typedef struct {
    map::MapPoint * mp;
    size_t edge_id;
    std::vector<MapPointMatch> matches;
  } MpContainer;
 protected:
  void ComputeBow();
  static void InitializeOptimizer(g2o::SparseOptimizer & optimizer) ;
  bool BaselineIsNotEnough(const MonocularFrame *other) const;
  void FindNewMapPointMatches(MonocularFrame * keyframe, std::unordered_map<std::size_t, std::size_t> & out_matches ) ;
  void ComputeMatches(MonocularFrame * reference_kf,
                      std::unordered_map<std::size_t, std::size_t> & out_matches,
                      bool self_keypoint_exists,
                      bool reference_kf_keypoint_exists);
  void ListMapPoints(std::unordered_set<map::MapPoint *> & out_map_points) const override;
 protected:

  features::Features features_;
  const std::shared_ptr<camera::MonocularCamera> camera_;
  // Feature id to MapPoint ptr
  std::map<size_t, map::MapPoint *> map_points_;

};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_MONOCULAR_FRAME_H_
