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
#include "tracking_result.h"
#include "local_mapper.h"

namespace orb_slam3 {

namespace serialization {
class SerializationContext;
}

class Tracker : public Observer<frame::KeyFrame *>,
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
  Tracker(orb_slam3::map::Atlas * atlas, LocalMapper * local_mapper);

  /*!
   * Destructor
   */
  virtual ~Tracker();

 public:
  /*!
   * Processes the frame
   * @param frame the next frame received from the sensor
   * @return Tracking result
   */
  TrackingResult Track(frame::Frame * frame);
  void SaveState(std::ostream & ostream);
  void LoadState(std::istream & istream, serialization::SerializationContext & context);
  map::Atlas * GetAtlas() const;
  /// Helper member functions
 private:
  TrackingResult TrackInOkState(frame::Frame * frame);
  TrackingResult TrackInFirstImageState(frame::Frame * frame);
  TrackingResult TrackInNotInitializedState(frame::Frame * frame);
  void PredictAndSetNewFramePosition(frame::Frame * frame) const;
  void ComputeVelocity(const geometry::RigidObject * frame2, const geometry::RigidObject * frame1);
  bool TrackWithMotionModel(frame::Frame * frame, std::list<frame::MapPointVisibilityParams> & out_visibles);
  bool TrackWithReferenceKeyFrame(frame::Frame * frame);
  void ReplaceLastFrame(frame::Frame * frame);
  bool NeedNewKeyFrame(frame::Frame * frame);
  void StartNewMap(frame::Frame * frame);

 private:
  static frame::KeyFrame * ListLocalKeyFrames(frame::Frame * current_frame,
                                              std::unordered_set<frame::KeyFrame *> & out_local_keyframes);

 private:
  /// Helper member variables
  int kf_counter_ = 0;
  map::Atlas * atlas_;
  bool velocity_is_valid_;

  geometry::Pose velocity_;

  frame::Frame * last_frame_;
  frame::KeyFrame * last_key_frame_;
  frame::KeyFrame * reference_keyframe_;
  State state_;
  size_t last_relocalization_frame_id_ = 0;
  LocalMapper * local_mapper_;
};

}
#endif //ORB_SLAM3_INCLUDE_TRACKER_H_
