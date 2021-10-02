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
  MonocularFrame(TimePoint time_point,
                 const std::string & filename,
                 const camera::MonocularCamera * camera,
                 const SensorConstants * sensor_constants,
                 const std::shared_ptr<features::handlers::BaseFeatureHandler> & handler);
  MonocularFrame(std::istream & stream, serialization::SerializationContext & context);
 public:
  // Frame
  FrameType Type() const override;
  bool IsValid() const override;
  KeyFrame * CreateKeyFrame() override;
  bool Link(Frame * other) override;
  bool FindMapPointsFromReferenceKeyFrame(const KeyFrame * reference_keyframe) override;
  bool EstimatePositionByProjectingMapPoints(Frame * frame, std::list<MapPointVisibilityParams> & out_visibles) override;
  void ListMapPoints(MapPointSet & out_map_points) const override;
  void OptimizePose() override;
  void FilterVisibleMapPoints(const std::unordered_set<map::MapPoint *> & map_points,
                              std::list<MapPointVisibilityParams> & out_filetered_map_points,
                              precision_t radius_multiplier) const override;
  void SearchInVisiblePoints(const std::list<MapPointVisibilityParams> & filtered_map_points) override;
  size_t GetMapPointsCount() const ;
  void UpdateFromReferenceKeyFrame() override;
  virtual void SerializeToStream(std::ostream & stream) const override;
 public:
  /*!
   * Used for debugging
   * @return a map of bad map points only
   */
  BaseMonocular::MonocularMapPoints GetBadMapPoints() const;
  const camera::ICamera * GetCamera() const override;
  void SetCamera(const camera::ICamera * icamera) override;
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
