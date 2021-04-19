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

namespace orb_slam3 {

using frame::FrameBase;
class Tracker : public Observer<std::shared_ptr<frame::FrameBase>>,
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
  TrackingResult Track(const std::shared_ptr<FrameBase> & frame);



  /*!
   * Destructor
   */
  virtual ~Tracker();

  /// Helper member functions
 private:
  TrackingResult TrackInOkState(const std::shared_ptr<FrameBase> & frame);
  TrackingResult TrackInFirstImageState(const std::shared_ptr<FrameBase> & frame);
  TrackingResult TrackInNotInitializedState(const std::shared_ptr<FrameBase> & frame);


  int kf_counter = 0;
  bool NeedNewKeyFrame();
  /// Helper member variables
 private:
  map::Atlas * atlas_;
  TVector3D velocity_;
  TMatrix33 angular_velocity_;
  std::shared_ptr<frame::FrameBase> last_frame_;
  std::shared_ptr<frame::FrameBase> last_key_frame_;
  std::shared_ptr<frame::FrameBase> initial_frame_;
  State state_;


};

}
#endif //ORB_SLAM3_INCLUDE_TRACKER_H_