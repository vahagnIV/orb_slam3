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
#include <frame/covisibility_graph_node.h>
#include <features/ifeature_extractor.h>
#include <geometry/pose.h>
#include <map/map_point.h>

namespace orb_slam3 {
namespace frame {

class FrameBase : public Identifiable {
 public:
  FrameBase() = delete;

  virtual FrameType Type() const = 0;

  FrameBase(const TimePoint & timestamp, const std::shared_ptr<features::IFeatureExtractor> & feature_extractor)
      : Identifiable(), timestamp_(timestamp), covisibility_connections_(), feature_extractor_(feature_extractor),
        initial_(false) {
  }
  /*!
   * Getter function
   * @return the value of timestamp associated with the frame instance
   */
  inline TimePoint TimeStamp() const noexcept { return timestamp_; }

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

  bool IsInitial() const { return initial_; }
  void SetInitial(bool initial) { initial_ = initial; }

  /*!
   * If needed, this should inizialize the position from the previous frames
   * Used only for monocular case
   * @return
   */
  virtual bool Link(const std::shared_ptr<FrameBase> & other) = 0;

  /*!
   * Return the map point associated with the id. The id is unique within the frame
   * @param id The id of the Map point in the frame
   * @return pointer to the map point
   */
  const map::MapPoint *MapPoint(size_t id) const;

  /*!
   * Non const method of the previous
   * @param id
   * @return
   */
//  map::MapPoint *&MapPoint(size_t id) const { return map_points_[id]; }

  /*!
   * Returns all map points associated with the frame
   * @return
   */
  std::map<size_t, map::MapPoint *> & MapPoints() { return map_points_; }
  const std::map<size_t, map::MapPoint *> & MapPoints() const { return map_points_; }

//  const std::vector<map::MapPoint * const> & MapPoints() const { return map_points_; }

  /*!
   * Appends the descriptors that correspond to the map_point with feature_id to the provided vector.
   * In case of multi-view frame 1 map point can correspond to multiple keypoints and, therefore, descriptors
   * @param feature_id The id of the keypoint
   * @param out_descriptor_ptr The vector to which the descriptors will be appended
   */
  virtual void AppendDescriptorsToList(size_t feature_id,
                                       std::vector<features::DescriptorType> & out_descriptor_ptr) const = 0;

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
  geometry::Pose *GetPose() { return &pose_; }

  /*!
   * Append necessary vertices and edges for BA
   * @param optimizer the g2o::Sparseoptimizer
   * @param next_id The nonce that is used to assign ids to the g2o objects
   */
  virtual void AppendToOptimizerBA(g2o::SparseOptimizer & optimizer, size_t & next_id) = 0;

  /*!
   * Collect the optimized values from the optimizer
   * @param optimizer
   */
  virtual void CollectFromOptimizerBA(g2o::SparseOptimizer & optimizer) = 0;

  /*!
   * Restores the position of the current frame with comparison to the reference keyframe.
   * Leves reference_keyframe untouches on fail.
   * @param reference_keyframe The reference keyframe
   * @return true on success
   */
  virtual bool TrackWithReferenceKeyFrame(const std::shared_ptr<FrameBase> & reference_keyframe) = 0;

  /*!
   * Non-const getter for the covisibility graph
   * @return
   */
  CovisibilityGraphNode & CovisibilityGraph() { return covisibility_connections_; }

  /*!
   * Creates new map points from the neighbouring frames
   */
  virtual void FindNewMapPoints() = 0;

  /*!
   * Computes the median of z coordinates of all visible map points
   * @return
   */
  virtual precision_t ComputeMedianDepth() const = 0;

  /*!
   * Destructor
   */
  virtual ~FrameBase();

  g2o::VertexSE3Expmap *CreatePoseVertex() const;
 protected:
  static void ListMapPoints(const std::unordered_set<FrameBase *> & frames, std::set<map::MapPoint *> & out_map_points);
  static void FixedFrames(const std::set<map::MapPoint *> & map_points,
                          const std::unordered_set<FrameBase *> & frames,
                          std::unordered_set<FrameBase *> & out_fixed_frames);

 protected:

  TimePoint timestamp_;

  // Feature id to MapPoint ptr
  std::map<size_t, map::MapPoint *> map_points_;

  CovisibilityGraphNode covisibility_connections_;

  // Transformation from the world coordinate system to the frame coordinate system
  geometry::Pose pose_;

  std::shared_ptr<features::IFeatureExtractor> feature_extractor_;

  bool initial_;

};

}
}

#endif //ORB_SLAM3_INCLUDE_FRAME_BASE_H_
