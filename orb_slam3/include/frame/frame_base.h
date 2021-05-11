//
// Created by vahagn on 11/29/20.
//

#ifndef ORB_SLAM3_INCLUDE_FRAME_BASE_H_
#define ORB_SLAM3_INCLUDE_FRAME_BASE_H_

// ===stl===
#include <memory>
#include <unordered_set>

// === g2o ===
#include <g2o/core/sparse_optimizer.h>

// == orb-slam3 ===
#include <typedefs.h>
#include <identifiable.h>
#include <frame/frame_type.h>
#include <frame/observation.h>
#include <frame/covisibility_graph_node.h>
#include <features/ifeature_extractor.h>
#include <geometry/pose.h>
#include <map/map_point.h>
#include <tracking_result.h>

namespace orb_slam3 {
namespace frame {

class FrameBase : public Identifiable {
 public:
  FrameBase() = delete;

  virtual FrameType Type() const = 0;

  FrameBase(const TimePoint & timestamp,
            const std::shared_ptr<features::IFeatureExtractor> & feature_extractor,
            const std::string & filename)
      : Identifiable(),
        frame_index_(++frame_counter_),
        timestamp_(timestamp),
        covisibility_connections_(this),
        feature_extractor_(feature_extractor),
        initial_(false), filename_(filename), is_keyfrme_(false) {
  }

  /*!
   * Getter function
   * @return the value of timestamp associated with the frame instance
   */
  inline TimePoint TimeStamp() const noexcept { return timestamp_; }

  inline const std::string & Filename() const { return filename_; }

  /*!
   * Initializes the position of the frame to identity, i.e. the frame is in the origin
   */
  void InitializeIdentity() noexcept;

  /*!
   * Set the position of the frame in the world coordinate system
   * @param pose 4x4 joint transformation matrix
   */
  void SetPosition(const geometry::Pose & pose) noexcept;

  /*!
   * Set the position of the frame in the world coordinate system
   * @param pose g2o Quaternion
   */
  void SetPosition(const geometry::Quaternion & pose) noexcept;

  /*!
   * If frame passed certain checks like the number of features etc
   * @return true if valid
   */
  virtual bool IsValid() const = 0;

  /*!
   * Checks if the map point is visible on this frame
   * @param map_point The map point to check
   * @return true if visible
   */

  bool IsInitial() const { return initial_; }
  void SetInitial(bool initial) { initial_ = initial; }

  /*!
   * If needed, this should initialize the position from the previous frames
   * Used only for monocular case
   * @return
   */
  virtual bool Link(FrameBase * other) = 0;

  /*!
   * Appends all local ma points to the output set
   * @param out_map_points
   */
  virtual void ListMapPoints(std::unordered_set<map::MapPoint *> & out_map_points) const = 0;

//  static void ListAllMapPoints(const std::unordered_set<FrameBase *> & frames,
//                               std::unordered_set<map::MapPoint *> & out_map_points);

  /*!
   * Computes the normal of the point.
   * @param point the point
   * @return The normal
   */
  virtual TVector3D GetNormal(const TPoint3D & point) const = 0;

  /*!
   * Getter for pose
   * @return pose
   */
  const geometry::Pose * GetPose() const { return &pose_; }

  /*!
   * Getter for pose
   * @return pose
   */
  const geometry::Pose * GetInversePose() const { return &inverse_pose_; }

  /*!
   * Restores the position of the current frame with comparison to the reference keyframe.
   * Leves reference_keyframe untouches on fail.
   * @param reference_keyframe The reference keyframe
   * @return true on success
   */
  virtual bool TrackWithReferenceKeyFrame(FrameBase * reference_keyframe) = 0;

  /*!
   * TODO: add description
   * @param map_points
   * @return
   */
  virtual bool FindNewMapPointsAndAdjustPosition(const std::unordered_set<map::MapPoint *> & map_points) = 0;

  /*!
   * Getter for the feature extractor
   * @return
   */
  const std::shared_ptr<features::IFeatureExtractor> & GetFeatureExtractor() const { return feature_extractor_; }

  /*!
   * Non-const getter for the covisibility graph
   * @return
   */
  CovisibilityGraphNode & CovisibilityGraph() { return covisibility_connections_; }

  /*!
   * Creates new map points by triangulating points between frames
   * @param other The other frame for triangulation
   * @param new_map_points Newly created map points that contain observations
   */
  virtual void CreateNewMapPoints(FrameBase * other) = 0;

  /*!
   * Computes the median of z coordinates of all visible map points
   * @return
   */
  virtual precision_t ComputeMedianDepth() const = 0;

  size_t GetIndex() const { return frame_index_; }

  virtual void AddMapPoint(Observation * observation) = 0;
  virtual void EraseMapPoint(map::MapPoint *) = 0;

  virtual void SearchLocalPoints(std::unordered_set<map::MapPoint *> & map_points) = 0;
  void SetKeyFrame(bool is_keyframe) { is_keyfrme_ = is_keyframe; }
  bool IsKeyFrame() const { return is_keyfrme_; }

  /*!
   * Destructor
   */
  virtual ~FrameBase();
  const std::unordered_set<Observation *> & GetRecentObservations() { return recent_observations_; }

 protected:

  static void FixedFrames(const std::unordered_set<map::MapPoint *> & map_points,
                          const std::unordered_set<FrameBase *> & frames,
                          std::unordered_set<FrameBase *> & out_fixed_frames);

 protected:
  std::unordered_set<Observation *> recent_observations_;
  static std::atomic_uint64_t frame_counter_;
  size_t frame_index_;

  TimePoint timestamp_;

  CovisibilityGraphNode covisibility_connections_;

  // Transformation from the world coordinate system to the frame coordinate system
  geometry::Pose pose_;

  // Transformation from the camera coordinate system to the world coordinate system
  geometry::Pose inverse_pose_;

  std::shared_ptr<features::IFeatureExtractor> feature_extractor_;

  bool initial_;

  const std::string filename_;

  bool is_keyfrme_;

};

typedef std::unordered_set<FrameBase *> FrameSet;

}
}

#endif //ORB_SLAM3_INCLUDE_FRAME_BASE_H_
