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
 private:
  explicit MonocularKeyFrame(MonocularFrame * frame);
 public:
  ~MonocularKeyFrame() override = default;
  bool IsVisible(map::MapPoint * map_point,
                 VisibleMapPoint & out_map_point,
                 precision_t radius_multiplier,
                 unsigned int window_size) const override;
 public:
  void CreateNewMapPoints(frame::KeyFrame * other) override;
  void ComputeBow() override;
  TVector3D GetNormal(const TPoint3D & point) const override;
  FrameType Type() const override;
  void ListMapPoints(MapPointSet & out_map_points) const override;
  void FuseMapPoints(MapPointSet & map_points) override;
 private:
  static bool BaseLineIsEnough(const MapPointSet & others_map_points,
                               const geometry::Pose & local_pose,
                               const geometry::Pose & others_pose);
  static precision_t ComputeBaseline(const geometry::Pose & local_pose, const geometry::Pose & others_pose);
  static precision_t ComputeSceneMedianDepth(const MapPointSet & map_points, unsigned q, const geometry::Pose & pose);
 private:
  void FilterVisibleMapPoints(const MapPointSet & map_points, std::list<VisibleMapPoint> & out_visibles);

};

}
}
}

#endif //ORB_SLAM3_SRC_FRAME_MONOCULAR_MONOCULAR_KEY_FRAME_H_
