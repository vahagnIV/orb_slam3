//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_KEY_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_KEY_FRAME_H_
#include "base_frame.h"
#include <features/features.h>
#include <frame/covisibility_graph_node.h>
#include <geometry/sim3_transformation.h>

namespace orb_slam3 {
namespace frame {

class Observation;

class KeyFrame : public BaseFrame {
 public:
  KeyFrame(TimePoint time_point,
           const std::string & filename,
           const SensorConstants * sensor_constants,
           size_t id,
           std::shared_ptr<const features::handlers::BaseFeatureHandler> feature_handler);

  virtual ~KeyFrame() = default;
 public:
  virtual TVector3D GetNormal(const TPoint3D & point) const = 0;
  CovisibilityGraphNode & GetCovisibilityGraph() {
    return covisibility_graph_;
  }
  const CovisibilityGraphNode & GetCovisibilityGraph() const {
    return covisibility_graph_;
  }
  bool IsInitial() const { return is_initial_; }
  void SetInitial(bool initial) { is_initial_ = initial; }
  bool IsBad() const { return bad_flag_; }
  virtual void SetBad();
  void SetPoseGBA(const TMatrix33 & R, const TVector3D & T);
  void SetKeyFrameGBA(KeyFrame * keyframe) { kf_gba_ = keyframe; }
  virtual void CreateNewMapPoints(frame::KeyFrame * other, MapPointSet & out_newly_created) = 0;
  virtual void FuseMapPoints(MapPointSet & map_points) = 0;
  virtual void EraseMapPoint(const map::MapPoint *) = 0;
  virtual void ReplaceMapPoint(map::MapPoint * map_point, const Observation & observation) = 0;
  typedef std::vector<std::pair<map::MapPoint *, map::MapPoint *>> MapPointMatches;
  virtual bool FindSim3Transformation(const MapPointMatches & map_point_matches,
                                      const KeyFrame * other,
                                      geometry::Sim3Transformation & out_transormation) const = 0;
  virtual void FindMatchingMapPoints(const KeyFrame * other,
                                     MapPointMatches & out_matches) const = 0;

  virtual void FilterVisibleMapPoints(const MapPointSet & map_points,
                                      const geometry::Sim3Transformation & relative_transformation,
                                      const geometry::Pose & mp_local_transformation,
                                      std::list<MapPointVisibilityParams> & out_visibles,
                                      precision_t radius_multiplier) const = 0;

  virtual size_t AdjustSim3Transformation(std::list<MapPointVisibilityParams> & visibles,
                                          const KeyFrame * relative_kf,
                                          geometry::Sim3Transformation & in_out_transformation) const = 0;

 protected:
  CovisibilityGraphNode covisibility_graph_;
  bool is_initial_;
  bool bad_flag_;

  // Container for saving the position after gba for LC
  geometry::Pose pose_gba_;
  KeyFrame * kf_gba_;

};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_KEY_FRAME_H_
