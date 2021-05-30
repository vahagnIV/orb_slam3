//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_MONOCULAR_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_MONOCULAR_FRAME_H_
#include <frame/frame.h>
#include "base_monocular.h"
#include "monocular_key_frame.h"

namespace orb_slam3 {
namespace frame {
namespace monocular {

class MonocularFrame : public Frame, public BaseMonocular {
 public:
  MonocularFrame(const TImageGray8U & image,
                 TimePoint time_point,
                 const string & filename,
                 const features::IFeatureExtractor * feature_extractor,
                 const camera::MonocularCamera * camera,
                 const features::BowVocabulary * vocabulary,
                 const SensorConstants * sensor_constants);
 public:
  // Frame
  FrameType Type() const override;
  bool IsValid() const override;
  KeyFrame * CreateKeyFrame() override;
  bool Link(Frame * other) override;
  bool FindMapPointsFromReferenceKeyFrame(const KeyFrame * reference_keyframe) override;
//  bool EstimatePositionByProjectingMapPoints(const list<VisibleMapPoint> & filtered_map_points) override;
  bool EstimatePositionByProjectingMapPoints(Frame * frame) override;
  void ListMapPoints(MapPointSet & out_map_points) const override;
  void ComputeBow() override;
  void OptimizePose() override;
  bool IsVisible(map::MapPoint * map_point,
                 VisibleMapPoint & out_map_point,
                 precision_t radius_multiplier,
                 unsigned int window_size) const ;
  void FilterVisibleMapPoints(const unordered_set<map::MapPoint *> & map_points,
                              list<VisibleMapPoint> & out_filetered_map_points,
                              precision_t radius_multiplier,
                              unsigned int window_size) const override;
  void SearchInVisiblePoints(const list<VisibleMapPoint> & filtered_map_points) override;
  size_t GetMapPointCount() const override;
  void UpdateFromReferenceKeyFrame() override;
 private:
  bool ComputeMatchesForLinking(MonocularFrame * from_frame, std::unordered_map<size_t, size_t> & out_matches) const;
  void InitializeMapPointsFromMatches(const std::unordered_map<std::size_t, std::size_t> & matches,
                                      const std::unordered_map<size_t, TPoint3D> & points,
                                      MonocularFrame * from_frame,
                                      MapPointSet & out_map_points);
  void ComputeMatchesFromReferenceKF(const orb_slam3::frame::monocular::MonocularKeyFrame * reference_kf,
                                     std::unordered_map<std::size_t, std::size_t> & out_matches,
                                     bool self_keypoint_exists,
                                     bool reference_kf_keypoint_exists) const;
  void FilterFromLastFrame(MonocularFrame * last_frame, std::list<VisibleMapPoint> & out_visibles,
                           precision_t radius_multiplier);

  void SearchInVisiblePoints(const std::list<VisibleMapPoint> & filtered_map_points, precision_t matcher_snn_threshold);
 public:
  // MonocularFame
 private:
  MonocularKeyFrame * reference_keyframe_;

};

}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_MONOCULAR_FRAME_H_
