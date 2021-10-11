//
// Created by vahagn on 30/06/2021.
//

#ifndef ORB_SLAM3_SRC_LOOP_MERGE_DETECTOR_H_
#define ORB_SLAM3_SRC_LOOP_MERGE_DETECTOR_H_

#include <frame/database/ikey_frame_database.h>
#include <map/atlas.h>
#include <concurrentqueue/concurrentqueue.h>
namespace orb_slam3 {
class LocalMapper;

enum DetectionType {
  LoopDetected = 1,
  MergeDetected = 2,
  Empty = 0
};

struct DetectionResult {
  DetectionType type;
  frame::KeyFrame * keyframe;
  frame::KeyFrame * candidate;
  geometry::Sim3Transformation transformation;
};

class LoopMergeDetector {

  friend class LocalMapper;
 public:
  LoopMergeDetector(map::Atlas * atlas);
  void RunIteration();
  ~LoopMergeDetector();
 public:
  void Start();
  void Stop();
  void Process(frame::KeyFrame * key_frame);
 private:
  void Run();

 private:
  typedef std::unordered_set<frame::KeyFrame *> KeyFrameSet;
  typedef std::vector<std::pair<map::MapPoint *, map::MapPoint *>> MapPointMatches;
  static bool DetectLoopOrMerge(const frame::KeyFrame * key_frame,
                                frame::IKeyFrameDatabase::KeyFrameSet & current_neighbourhood,
                                frame::KeyFrame * candidate_keyframe,
                                geometry::Sim3Transformation & out_sim3_transformation);
  void SetLocalMapper(LocalMapper * local_mapper);
 private:
  static bool Intersect(const KeyFrameSet & bow_candidate_neighbours, const KeyFrameSet & key_frame_neighbours);
  static void FindMapPointMatches(const frame::KeyFrame * current_key_frame,
                                  const KeyFrameSet & loop_neighbours,
                                  MapPointMatches & out_matches);
  static void ListSurroundingWindow(const frame::KeyFrame * key_frame,
                                    frame::IKeyFrameDatabase::KeyFrameSet & out_window);
  static void ListAllMapPoints(const frame::IKeyFrameDatabase::KeyFrameSet & key_frames,
                               std::unordered_set<map::MapPoint *> & out_mps);
 private:
  map::Atlas * atlas_;
  bool canceled_;
  moodycamel::ConcurrentQueue<frame::KeyFrame *> queue_;
  std::thread * thread_;
  LocalMapper * local_mapper_;

};

}

#endif //ORB_SLAM3_SRC_LOOP_MERGE_DETECTOR_H_
