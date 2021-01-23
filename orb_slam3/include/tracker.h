//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_TRACKER_H_
#define ORB_SLAM3_INCLUDE_TRACKER_H_
#include <frame/frame_base.h>
#include <atlas.h>
#include <memory>
namespace orb_slam3 {

enum TrackingResult {
  OK, OldFrame, Ignore
};

using frame::FrameBase;
class Tracker {
 public:
  Tracker();

  /*!
   * Processes the frame
   * @param frame the next frame received from the sensor
   * @return Tracking result
   */
  TrackingResult Track(std::shared_ptr<FrameBase> frame);

  /*!
   * Destructor
   */
  virtual ~Tracker();

 private:

  bool TrackReferenceKeyFrame();

 private:
  Atlas * atlas_;


};

}
#endif //ORB_SLAM3_INCLUDE_TRACKER_H_
