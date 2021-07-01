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
  bool IsVisible(map::MapPoint * map_point,
                 MapPointVisibilityParams & out_map_point,
                 precision_t radius_multiplier,
                 unsigned int window_size) const ;

  void CreateNewMapPoints(frame::KeyFrame * other, MapPointSet & out_newly_created) override;
  TVector3D GetNormal(const TPoint3D & point) const override;
  FrameType Type() const override;
  void ListMapPoints(MapPointSet & out_map_points) const override;
  precision_t GetSimilarityScore(const BaseFrame * other) const override;
  void FuseMapPoints(MapPointSet & map_points) override;
  void EraseMapPoint(const map::MapPoint *) override;
  void ReplaceMapPoint(map::MapPoint * map_point, const Observation & observation) override;
 protected:
  void SerializeToStream(ostream & stream) const override;
 public:
  void AddMapPoint(map::MapPoint * map_point, size_t feature_id) override;
  void EraseMapPoint(size_t feature_id) override;
  void SetBad() override;
 private:

  void FilterVisibleMapPoints(const MapPointSet & map_points, std::list<MapPointVisibilityParams> & out_visibles);
  void EraseMapPointImpl(const map::MapPoint *, bool check_bad);
  void EraseMapPointImpl(size_t feature_id, bool check_bad) ;

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
