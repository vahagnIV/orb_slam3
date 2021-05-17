//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_TRACKER_H_
#define ORB_SLAM3_INCLUDE_TRACKER_H_

// == stl ===
#include <memory>
#include <unordered_set>

// == orb-slam3 ===
#include "frame/frame.h"
#include "map/atlas.h"
#include "observable.h"
#include "position_observer.h"
#include "local_mapper.h"

namespace orb_slam3 {

using frame::FrameBase;
class Tracker : public Observer<frame::FrameBase *>,
                public Observable<UpdateMessage> {
 public:
  enum State {
    NOT_INITIALIZED,
    LOST,
    RECENTLY_LOST,
    FIRST_IMAGE,
    OK
  };
 public:
  Tracker(orb_slam3::map::Atlas * atlas);

  /*!
   * Processes the frame
   * @param frame the next frame received from the sensor
   * @return Tracking result
   */
  TrackingResult Track(frame::Frame * frame);

  /*!
   * Destructor
   */
  virtual ~Tracker();

  /// Helper member functions
 private:
  TrackingResult TrackInOkState(frame::Frame * frame);
  TrackingResult TrackInFirstImageState(frame::Frame * frame);
  TrackingResult TrackInNotInitializedState(frame::Frame * frame);
  void PredictAndSetNewFramePosition(frame::Frame * frame) const;
  void ComputeVelocity(const geometry::RigidObject * frame2, const geometry::RigidObject * frame1);
  void UpdateLocalMap(frame::Frame * current_frame);
  void UpdateLocalKeyFrames(frame::Frame * current_frame);
  void UpdateLocalPoints();
  void UpdateCovisibilityConnections();
  bool TrackWithMotionModel(frame::Frame * frame);
  bool TrackWithReferenceKeyFrame(frame::Frame * frame);
  void OptimizePose(FrameBase * frmae);
  void ReplaceLastFrame(frame::Frame * frame);

  bool NeedNewKeyFrame(frame::Frame * frame);
 private:
  /// Helper member variables
  LocalMapper local_mapper_;
  int kf_counter = 0;
  map::Atlas * atlas_;
  bool velocity_is_valid_;
  TVector3D velocity_;
  TMatrix33 angular_velocity_;
  frame::Frame * last_frame_;
  frame::KeyFrame * last_key_frame_;
  frame::KeyFrame * reference_keyframe_;
  std::unordered_set<map::MapPoint *> local_map_points_;
  std::unordered_set<frame::KeyFrame *> local_key_frames_;
  State state_;

};

}
#endif //ORB_SLAM3_INCLUDE_TRACKER_H_