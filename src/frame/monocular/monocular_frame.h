//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_MONOCULAR_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_MONOCULAR_FRAME_H_
#include <frame/frame.h>
#include "base_monocular.h"
#include "monocular_key_frame.h"
#include <features/factories/handler_factory.h>

namespace orb_slam3 {
namespace frame {
namespace monocular {

class MonocularFrame : public Frame, public BaseMonocular {
 public:
  MonocularFrame(const TImageGray8U & image,
                 TimePoint time_point,
                 const std::string & filename,
                 const features::IFeatureExtractor * feature_extractor,
                 const camera::MonocularCamera * camera,
                 const SensorConstants * sensor_constants,
                 const features::HandlerFactory * handler_factory);
 public:
  // Frame
  FrameType Type() const override;
  bool IsValid() const override;
  KeyFrame * CreateKeyFrame() override;
  bool Link(Frame * other) override;
  bool FindMapPointsFromReferenceKeyFrame(const KeyFrame * reference_keyframe) override;
  bool EstimatePositionByProjectingMapPoints(Frame * frame, std::list<MapPointVisibilityParams> & out_visibles) override;
  void ListMapPoints(MapPointSet & out_map_points) const override;
  precision_t GetSimilarityScore(const BaseFrame * other) const override;
  void OptimizePose() override;
  void FilterVisibleMapPoints(const std::unordered_set<map::MapPoint *> & map_points,
                              std::list<MapPointVisibilityParams> & out_filetered_map_points,
                              precision_t radius_multiplier) const override;
  void SearchInVisiblePoints(const std::list<MapPointVisibilityParams> & filtered_map_points) override;
  size_t GetMapPointCount() const override;
  void UpdateFromReferenceKeyFrame() override;
  BaseMonocular::MonocularMapPoints GetBadMapPoints() const;
 protected:
  void SerializeToStream(std::ostream & stream) const override;

 private:
  bool ComputeMatchesForLinking(MonocularFrame * from_frame, std::unordered_map<size_t, size_t> & out_matches) const;
  void InitializeMapPointsFromMatches(const std::unordered_map<std::size_t, std::size_t> & matches,
                                      const std::unordered_map<size_t, TPoint3D> & points,
                                      MonocularFrame * from_frame,
                                      MapPointSet & out_map_points);
  void ComputeMatchesFromReferenceKF(const MonocularKeyFrame * reference_kf,
                                     std::unordered_map<std::size_t, std::size_t> & out_matches) const;
  void FilterFromLastFrame(MonocularFrame * last_frame, std::list<MapPointVisibilityParams> & out_visibles,
                           precision_t radius_multiplier);

  void SearchInVisiblePoints(const std::list<MapPointVisibilityParams> & filtered_map_points, precision_t matcher_snn_threshold);
 public:
  // MonocularFame
 private:
  MonocularKeyFrame * reference_keyframe_;

};

}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_MONOCULAR_FRAME_H_
