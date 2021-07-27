//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_SRC_FRAME_MONOCULAR_MONOCULAR_KEY_FRAME_H_
#define ORB_SLAM3_SRC_FRAME_MONOCULAR_MONOCULAR_KEY_FRAME_H_
#include <frame/key_frame.h>
#include "base_monocular.h"

namespace orb_slam3 {
namespace frame {
namespace monocular {
class MonocularFrame;

class MonocularKeyFrame : public KeyFrame, public BaseMonocular {

  friend class MonocularFrame;

  /// Special member functions
 private:
  explicit MonocularKeyFrame(MonocularFrame * frame);

 public:
  ~MonocularKeyFrame() override = default;

 public:

  void CreateNewMapPoints(frame::KeyFrame * other, MapPointSet & out_newly_created) override;
  TVector3D GetNormal(const TPoint3D & point) const override;
  FrameType Type() const override;
  void ListMapPoints(MapPointSet & out_map_points) const override;
  void FuseMapPoints(MapPointSet & map_points) override;
  void EraseMapPoint(const map::MapPoint *) override;
  void ReplaceMapPoint(map::MapPoint * map_point, const Observation & observation) override;
 protected:
  void SerializeToStream(std::ostream & stream) const override;
 public:
  void AddMapPoint(map::MapPoint * map_point, size_t feature_id) override;
  void EraseMapPoint(size_t feature_id) override;
  void SetBad() override;
  void FindMatchingMapPoints(const KeyFrame * other,
                             MapPointMatches & out_matches) const override;
  bool FindSim3Transformation(const MapPointMatches & map_point_matches,
                              const KeyFrame * loop_candidate,
                              geometry::Sim3Transformation & out_transormation) const override;
  void FilterVisibleMapPoints(const MapPointSet & map_points,
                              const geometry::Sim3Transformation & relative_transformation,
                              const geometry::Pose & mp_local_transformation,
                              std::list<MapPointVisibilityParams> & out_visibles,
                              precision_t radius_multiplier) const override;
  size_t AdjustSim3Transformation(std::list<MapPointVisibilityParams> & visibles,
                                  const KeyFrame * relative_kf,
                                  geometry::Sim3Transformation & in_out_transformation) const override;
 private:

  void FilterVisibleMapPoints(const MapPointSet & map_points, std::list<MapPointVisibilityParams> & out_visibles);
  void EraseMapPointImpl(const map::MapPoint *, bool check_bad);
  void EraseMapPointImpl(size_t feature_id, bool check_bad);
  int GetMapPointLevel(const map::MapPoint * map_point) const;

 private:
  static bool BaseLineIsEnough(const MapPointSet & others_map_points,
                               const geometry::Pose & local_pose,
                               const geometry::Pose & others_pose);
  static precision_t ComputeBaseline(const geometry::Pose & local_pose, const geometry::Pose & others_pose);
  static precision_t ComputeSceneMedianDepth(const MapPointSet & map_points, unsigned q, const geometry::Pose & pose);

};

}
}
}

#endif //ORB_SLAM3_SRC_FRAME_MONOCULAR_MONOCULAR_KEY_FRAME_H_
