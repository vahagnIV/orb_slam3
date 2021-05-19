//
// Created by vahagn on 16/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_LOCAL_MAPPER_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_LOCAL_MAPPER_H_

// === stl ===
#include <thread>

// === orb-slam3 ===
#include "position_observer.h"
#include "observable.h"
#include "map/atlas.h"
namespace orb_slam3 {

class LocalMapper : public PositionObserver,
                    public Observable<frame::KeyFrame *> {
 public:
  explicit LocalMapper(map::Atlas * atlas);
  ~LocalMapper() override;
 public:
  void Start();
  void Stop();
  void MapPointCulling(frame::KeyFrame * keyframe);
  void RunIteration();
 private:
  void Run();
  void ProcessNewKeyFrame(frame::KeyFrame * frame);
  bool CheckNewKeyFrames() const;
 private:
  static void FilterFixedKeyFames(std::unordered_set<frame::KeyFrame *> & local_keyframes,
                                  frame::KeyFrame::MapPointSet & local_map_points,
                                  std::unordered_set<frame::KeyFrame *> & out_fixed) ;
  static void CreateNewMapPoints(frame::KeyFrame * key_frame) ;
  static void Optimize(frame::KeyFrame * frame);

 private:
  std::unordered_set<map::MapPoint *> recently_added_map_points_;
  map::Atlas * atlas_;
  std::atomic_bool cancelled_;
  std::thread * thread_;
};

}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_LOCAL_MAPPER_H_
