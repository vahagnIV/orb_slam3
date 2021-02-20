//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_TRACKER_H_
#define ORB_SLAM3_INCLUDE_TRACKER_H_

// == stl ===
#include <memory>

// == orb-slam3 ===
#include <frame/frame_base.h>
#include <map/atlas.h>

namespace orb_slam3 {

enum TrackingResult {
  OK, OldFrame, Ignore
};



using frame::FrameBase;
class Tracker {
 public:
 enum State {
    NOT_INITIALIZED,
    FIRST_IMAGE,
    OK
  };
 public: 
  Tracker();

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

 private:
  bool TrackReferenceKeyFrame();
  
 private:  
  map::Atlas * atlas_;
  std::shared_ptr<frame::FrameBase> last_frame_;
  std::shared_ptr<frame::FrameBase> initial_frame_;
  State state_;


};

}
#endif //ORB_SLAM3_INCLUDE_TRACKER_H_
