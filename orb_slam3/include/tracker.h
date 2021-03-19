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
#include <position_observer.h>

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
  Tracker(orb_slam3::map::Atlas * atlas);

  /*!
   * Processes the frame
   * @param frame the next frame received from the sensor
   * @return Tracking result
   */
  TrackingResult Track(const std::shared_ptr<FrameBase> & frame);

  /*!
 * Add an observer
 * @param observer
 */
  void AddObserver(PositionObserver * observer){
    observers_.insert(observer);
  }

  /*!
   * Remove the observer
   * @param observer
   */
  void RemoveObserver(PositionObserver * observer){
    observers_.erase(observer);
  }

  /*!
   * Destructor
   */
  virtual ~Tracker();

 private:
  bool TrackReferenceKeyFrame();
  void NotifyObservers(const std::shared_ptr<const FrameBase> & frame, MessageType type);
  
 private:  
  map::Atlas * atlas_;
  std::shared_ptr<frame::FrameBase> last_frame_;
  std::shared_ptr<frame::FrameBase> initial_frame_;
  State state_;
  std::unordered_set<PositionObserver *> observers_;


};

}
#endif //ORB_SLAM3_INCLUDE_TRACKER_H_
