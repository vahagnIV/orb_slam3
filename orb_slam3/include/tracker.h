//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_TRACKER_H_
#define ORB_SLAM3_INCLUDE_TRACKER_H_

// == stl ===
#include <memory>
#include <unordered_set>

// == orb-slam3 ===
#include <frame/frame_base.h>
#include <map/atlas.h>
#include <observable.h>
#include <position_observer.h>
#include <local_mapper.h>

namespace orb_slam3 {

using frame::FrameBase;
class Tracker : public Observer<frame::FrameBase*>,
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
  TrackingResult Track(FrameBase * frame);

  /*!
   * Destructor
   */
  virtual ~Tracker();

  /// Helper member functions
 private:
  TrackingResult TrackInOkState( FrameBase *  frame);
  TrackingResult TrackInFirstImageState( FrameBase * frame);
  TrackingResult TrackInNotInitializedState( FrameBase * frame);
  void PredictAndSetNewFramePosition( FrameBase * frame) const;
  void ComputeVelocity(const FrameBase * frame2, const FrameBase * frame1);
  void UpdateLocalMap( frame::FrameBase * current_frame);
  void UpdateLocalKeyFrames( frame::FrameBase * current_frame);
  void UpdateLocalPoints();
  void UpdateCovisibilityConnections();

  bool NeedNewKeyFrame(frame::FrameBase * frame);
 private:
  /// Helper member variables
  LocalMapper local_mapper_;
  int kf_counter = 0;
  map::Atlas * atlas_;
  TVector3D velocity_;
  TMatrix33 angular_velocity_;
  frame::FrameBase * last_frame_;
  frame::FrameBase * last_key_frame_;
  frame::FrameBase * initial_frame_;
  frame::FrameBase * reference_keyframe_;
  std::unordered_set<map::MapPoint *> local_map_points_;
  std::unordered_set<frame::FrameBase *> local_key_frames_;
  State state_;

};

}
#endif //ORB_SLAM3_INCLUDE_TRACKER_H_