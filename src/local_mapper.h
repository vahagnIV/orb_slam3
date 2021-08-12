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
#include <frame/database/ikey_frame_database.h>

namespace orb_slam3 {

class LocalMapper {
 public:
  explicit LocalMapper(map::Atlas * atlas, frame::IKeyFrameDatabase * key_frame_database);
  ~LocalMapper();
 public:
  void Start();
  void Stop();
  void RunIteration();
  bool AcceptKeyFrames() { return accept_key_frames_; };
  void AddToqueue(frame::KeyFrame * key_frame);
  size_t GetQueueSize() const { return new_key_frames_.size_approx(); }
 private:
  void Run();
  void MapPointCulling(frame::KeyFrame * keyframe);
  void ProcessNewKeyFrame(frame::KeyFrame * frame);
  bool CheckNewKeyFrames() const;
  void CreateNewMapPoints(frame::KeyFrame * key_frame);
  void KeyFrameCulling(frame::KeyFrame * keyframe);

 private:
  static void FilterFixedKeyFames(std::unordered_set<frame::KeyFrame *> & local_keyframes,
                                  frame::KeyFrame::MapPointSet & local_map_points,
                                  std::unordered_set<frame::KeyFrame *> & out_fixed);
  static void Optimize(frame::KeyFrame * frame);
  static void ListCovisiblesOfCovisibles(frame::KeyFrame * frame, std::unordered_set<frame::KeyFrame *> & out);
  static void FuseMapPoints(frame::KeyFrame * frame);
  static void ReplaceMapPoint(map::MapPoint * old_mp, map::MapPoint * new_mp);
  static void SetBad(map::MapPoint * map_point);

 private:
  moodycamel::BlockingConcurrentQueue<frame::KeyFrame *> new_key_frames_;
  std::unordered_set<map::MapPoint *> recently_added_map_points_;
  map::Atlas * atlas_;
  std::atomic_bool cancelled_;
  std::thread * thread_;
  bool accept_key_frames_;
  frame::IKeyFrameDatabase * key_frame_database_;
};

}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_LOCAL_MAPPER_H_
