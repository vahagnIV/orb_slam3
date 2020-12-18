//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_TRACKER_H_
#define ORB_SLAM3_INCLUDE_TRACKER_H_
#include <frame_base.h>
#include <atlas.h>
#include <memory>
namespace nvision {

enum TrackingResult {
  OK, OldFrame, Ignore
};

class Tracker {
 public:
  Tracker();

  /*!
   * Processes the frame
   * @param frame the next frame received from the sensor
   * @return Tracking result
   */
  TrackingResult Track(FrameBase *frame);

  virtual ~Tracker();

 private:
  Atlas *atlas_;
  T3DVector velocity_;

};

}
#endif //ORB_SLAM3_INCLUDE_TRACKER_H_
