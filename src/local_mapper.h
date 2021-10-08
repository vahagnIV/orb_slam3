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
#include "loop_merge_detector.h"

namespace orb_slam3 {
//class LoopMergeDetector;
//class DetectionResult;

class LocalMapper {
 public:
  LocalMapper(map::Atlas * atlas, LoopMergeDetector * loop_mege_detector);
  ~LocalMapper();
 public:
  /*!
   * Start the local mapper. Starts a new thread.
   */
  void Start();

  /*!
   * Stop the local mapper.
   */
  void Stop();

  /*!
   * Run single iteration. This function is public only for single threaded build
   */
  void RunIteration();

  /*!
   * Whether Local mapper can accept a new keyframe
   * @return true if it can
   */
  bool AcceptKeyFrames() const { return accept_key_frames_; };

  /*!
   * Add a keyframe to the local mapper's queue for further processin
   * @param key_frame keyframe to be processed
   */
  void AddToQueue(frame::KeyFrame * key_frame);

  /*!
   * Returns an approximate size of local mapper's queue
   * @return The approximate size of the queue
   */
  size_t GetQueueSize() const { return new_key_frames_.size_approx(); }

  void ProcessDetection(const DetectionResult & detection_result);
 private:
  void Run();
  void MapPointCulling(frame::KeyFrame * keyframe);
  void ProcessNewKeyFrame(frame::KeyFrame * frame);
  bool CheckNewKeyFrames() const;
  void CreateNewMapPoints(frame::KeyFrame * key_frame);
  void KeyFrameCulling(frame::KeyFrame * keyframe);

 private:
  /*!
   * Chooses keyframes which position should remain fixed during bundle adjustment
   * @param local_keyframes Keyframes from which to choose
   * @param local_map_points The local map point to be considered
   * @param out_fixed The output set of the fixed keyframes
   */
  static void FilterFixedKeyFames(const std::unordered_set<frame::KeyFrame *> & local_keyframes,
                                  const frame::BaseFrame::MapPointSet & local_map_points,
                                  std::unordered_set<frame::KeyFrame *> & out_fixed);

  // TODO: Rename this function
  static void Optimize(frame::KeyFrame * frame);

  /*!
   * List all neighbours and their neighbours of this keyframe
   * @param frame The keyframe
   * @param out The list of neighbour keyframes up to the second order
   */
  static void ListCovisiblesOfCovisibles(const frame::KeyFrame * frame,
                                         std::unordered_set<frame::KeyFrame *> & out);

  /*!
   * Find map points that represent the same physical point and fuse them,
   * i.e. pass all observation of one to the other and remove the first one.
   * @param frame The keyframe which map points to consider
   */
  static void FuseMapPoints(frame::KeyFrame * frame);

  /*!
   * Replace the old map point with the new one everywhere
   * @param old_mp The old map point
   * @param new_mp The new map point
   */
  static void ReplaceMapPoint(map::MapPoint * old_mp, map::MapPoint * new_mp);

  /*!
   * Set bad flag of the map point and remove it from everywhere
   * @param map_point
   */
  static void SetBad(map::MapPoint * map_point);

 private:
  moodycamel::BlockingConcurrentQueue<frame::KeyFrame *> new_key_frames_;
  std::unordered_set<map::MapPoint *> recently_added_map_points_;
  map::Atlas * atlas_;
  std::atomic_bool cancelled_;
  std::thread * thread_;
  bool accept_key_frames_;
  LoopMergeDetector * loop_merge_detector_;
};

}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_LOCAL_MAPPER_H_
