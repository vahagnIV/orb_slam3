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
                    public Observable<frame::FrameBase *> {
 public:
  LocalMapper(map::Atlas * atlas);
  void Start();
  void Stop();
  ~LocalMapper();
  void Optimize(frame::KeyFrame * frame);
  void MapPointCulling(frame::KeyFrame * keyframe);
  void RunIteration();
 private:
  static void EraseMapPoint(map::MapPoint * map_point);
  void Run();
  void ProcessNewKeyFrame(frame::KeyFrame * frame);
  void CreateNewMapPoints(frame::KeyFrame * key_frame) const ;
  bool CheckNewKeyFrames() const;
  void FilterFixedKeyFames(const std::unordered_set<frame::KeyFrame *> & keyframes ,std::unordered_set<frame::KeyFrame *> & out_fixed) const;
 private:
  std::unordered_set<map::MapPoint *> recently_added_map_points_;
  map::Atlas * atlas_;
  std::atomic_bool cancelled_;
  std::thread * thread_;
};

}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_LOCAL_MAPPER_H_
