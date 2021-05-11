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
#include <frame/visible_map_point.h>
#include <camera/monocular_camera.h>
#include <features/bow_vocabulary.h>
#include <optimization/edges/se3_project_xyz_pose.h>

namespace orb_slam3 {
namespace frame {
class MonocularObservation;
namespace test {
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
                 features::BowVocabulary * vocabulary);

  // ==== FrameBase =========
  bool IsValid() const override;
  FrameType Type() const override { return MONOCULAR; }
  bool Link(FrameBase * other) override;

  TVector3D GetNormal(const TVector3D & point) const override;
  bool TrackWithReferenceKeyFrame(FrameBase * reference_keyframe) override;
  bool FindNewMapPointsAndAdjustPosition(const std::unordered_set<map::MapPoint *> & map_points) override;
  const features::Features & GetFeatures() const { return features_; }
  void ListMapPoints(unordered_set<map::MapPoint *> & out_map_points) const override;
  void EraseMapPoint(map::MapPoint * map_point) override;

  // ==== Monocular ====
  const std::shared_ptr<camera::MonocularCamera> & GetCamera() const { return camera_; }
  const std::map<std::size_t, map::MapPoint *> & GetMapPoints() const {return map_points_; }
  void OptimizePose(std::unordered_set<std::size_t> & out_inliers);
  precision_t ComputeMedianDepth() const override;
  ~MonocularFrame();
  void SearchLocalPoints(unordered_set<map::MapPoint *> & map_points) override;
 protected:

  inline bool IsVisible(map::MapPoint * map_point,
                        VisibleMapPoint & out_map_point,
                        precision_t radius_multiplier = 1,
                        unsigned window_size = 0) const;

 public:
  void CreateNewMapPoints(FrameBase * other) override;
  void AddMapPoint(Observation * observation) override;
 protected:
  bool ComputeMatchesForLinking(MonocularFrame * from_frame, std::unordered_map<size_t, size_t> & out_matches);
  void InitializeMapPointsFromMatches(const unordered_map<std::size_t, std::size_t> & matches,
                                      const std::unordered_map<size_t, TPoint3D> & points,
                                      MonocularFrame * from_frame,
                                      unordered_set<map::MapPoint *> & out_map_points);

  void FilterVisibleMapPoints(const std::unordered_set<map::MapPoint *> map_points,
                              std::list<VisibleMapPoint> & out_filetered_map_points,
                              precision_t radius_multiplier = 1,
                              unsigned window_size = 0) const;

  void FindCandidateMapPointMatchesByProjection(const std::list<VisibleMapPoint> & filtered_map_points,
                                                std::unordered_map<map::MapPoint *, std::size_t> & out_matches);
  bool MapPointExists(const map::MapPoint * map_point) const;
  void ComputeBow();
  bool BaselineIsNotEnough(const MonocularFrame * other) const;
  void CreateNewMpPoints(MonocularFrame * frame,
                         const std::unordered_map<std::size_t, std::size_t> & matches,
                         std::unordered_set<map::MapPoint *> & out_map_points);
  static optimization::edges::SE3ProjectXYZPose * CreateEdge(map::MapPoint * map_point, MonocularFrame * frame);
  void ComputeMatches(MonocularFrame * reference_kf,
                      std::unordered_map<std::size_t, std::size_t> & out_matches,
                      bool self_keypoint_exists,
                      bool reference_kf_keypoint_exists);

  template<typename T>
  void SetDiff(const std::unordered_set<T> & set1, const std::unordered_set<T> & set2, std::unordered_set<T> & result) {
    for (auto & elem: set1) {
      if (set2.find(elem) == set2.end())
        result.insert(elem);
    }
  }
 protected:
  features::Features features_;
  std::map<std::size_t , map::MapPoint *> map_points_;
  const std::shared_ptr<camera::MonocularCamera> camera_;

};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_MONOCULAR_FRAME_H_
