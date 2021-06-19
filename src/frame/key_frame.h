//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_KEY_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_KEY_FRAME_H_
#include "base_frame.h"
#include <features/features.h>
#include <frame/covisibility_graph_node.h>

namespace orb_slam3 {
namespace frame {

class Observation;

class KeyFrame : public BaseFrame {
 public:
  KeyFrame(TimePoint time_point,
           const std::string & filename,
           const features::IFeatureExtractor * feature_extractor,
           const features::BowVocabulary * vocabulary,
           const SensorConstants * sensor_constants,
           size_t id) : BaseFrame(time_point, filename, feature_extractor, vocabulary, sensor_constants, id),
                        covisibility_graph_(this),
                        is_initial_(false),
                        bad_flag_(false),
                        kf_gba_(nullptr) {}

  virtual ~KeyFrame() = default;
 public:
  virtual TVector3D GetNormal(const TPoint3D & point) const = 0;
  CovisibilityGraphNode & GetCovisibilityGraph() {
    return covisibility_graph_;
  }
  bool IsInitial() const { return is_initial_; }
  void SetInitial(bool initial) { is_initial_ = initial; }
  bool IsBad() const { return bad_flag_; }
  virtual void SetBad() {
    bad_flag_ = true;
  }
  void SetPoseGBA(const TMatrix33 & R, const TVector3D & T) {
    pose_gba_.R = R;
    pose_gba_.T = T;
  }
  void SetKeyFrameGBA(KeyFrame * keyframe) {
    kf_gba_ = keyframe;
  }
  virtual void CreateNewMapPoints(frame::KeyFrame * other, MapPointSet & out_newly_created) = 0;
  virtual void FuseMapPoints(MapPointSet & map_points) = 0;
  virtual void EraseMapPoint(const map::MapPoint *) = 0;
  virtual void ReplaceMapPoint(map::MapPoint * map_point, const Observation & observation) = 0;

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
