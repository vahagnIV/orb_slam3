//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_KEY_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_KEY_FRAME_H_
#include "base_frame.h"
#include <features/features.h>
#include <frame/covisibility_graph_node.h>
#include <geometry/sim3_transformation.h>

namespace orb_slam3 {
namespace frame {

class Observation;

class KeyFrame : public BaseFrame {
 public:
  typedef std::vector<std::pair<map::MapPoint *, map::MapPoint *>> MapPointMatches;
  typedef std::vector<std::pair<Observation, Observation>> NewMapPoints;
  KeyFrame(std::istream &stream, serialization::SerializationContext &context);

  KeyFrame(TimePoint time_point,
           const std::string &filename,
           const SensorConstants *sensor_constants,
           size_t id,
           map::Atlas *atlas,
           map::Map *map,
           const geometry::Pose &pose);

  virtual ~KeyFrame() = default;

 public:

  void Initialize();

  /*!
   * Calculates the normal of a point to the origin of the local coordinate frame
   * @param point the point
   * @return the normal
   */
  virtual TVector3D GetNormal(const TPoint3D & point) const = 0;

  /*!
   * Same as "GetNormal" however uses the staging variables
   * @param point
   * @return
   */
  virtual TVector3D GetNormalFromStaging(const TPoint3D & point) const = 0;

  /*!
   * Getter for the covisibility graph
   * @return
   */
  CovisibilityGraphNode & GetCovisibilityGraph();

  /*!
   * Const getter for the covisibility graph
   * @return
   */
  const CovisibilityGraphNode & GetCovisibilityGraph() const;

  /*!
   * Checks if the frame is the initial frame in the map
   * @return true is initial
   */
  bool IsInitial() const;

  /*!
   * Filters visible map points from map_points
   * @param map_points container of initial map points
   * @param out_visibles output container of visible map points
   * @param use_staging use staging coordinates
   */
  virtual void FilterVisibleMapPoints(const BaseFrame::MapPointSet & map_points,
                                      std::list<MapPointVisibilityParams> & out_visibles,
                                      bool use_staging) const = 0;

  /*!
 * Creates new map points by matching features between the current frame and the "other"
 * @param other The other frame
 * @param out_newly_created output set of newly created map points
 */
  virtual void CreateNewMapPoints(frame::KeyFrame * other, NewMapPoints & out_newly_created) const = 0;

  /*!
   * Searches visible map points for matching with local features
   * @param visibles The input visibles
   * @param out_matched_map_points the map points that matched each other
   * @param out_local_matches map points that matched to features but the current does not have a map point associated to it
   */
  virtual void MatchVisibleMapPoints(const std::list<MapPointVisibilityParams> & visibles,
                                     std::list<std::pair<map::MapPoint *, map::MapPoint *>> & out_matched_map_points,
                                     std::list<Observation> & out_local_matches) const = 0;

  /*!
   * Sets the initial flag to true
   * @param initial
   */
  void SetInitial(bool initial);

  /*!
   * Checks if the keyframe should be considered bad
   * @return true if bad
   */
  bool IsBad() const;

  /*!
   * Sets the bad flag and removes the keyframe from the graph
   */
  virtual void SetBad();

  void SetPoseGBA(const TMatrix33 & R, const TVector3D & T);
  void SetKeyFrameGBA(KeyFrame * keyframe) { kf_gba_ = keyframe; }

  /*!
   * Adds map point to the key-frame's container
   * @param observation
   */
  virtual void AddMapPoint(Observation & observation);

  /*!
   * Erases the map point from the current frame
   */
  virtual void EraseMapPoint(map::MapPoint *);

  /*!
   * Locks the map point container so map points can be added and erased safely
   */
  virtual void LockMapPointContainer() const = 0;

  /*!
   * Releases the lock acquired by call of LockMapPointContainer
   */
  virtual void UnlockMapPointContainer() const = 0;

  virtual bool FindSim3Transformation(const MapPointMatches & map_point_matches,
                                      const KeyFrame * other,
                                      geometry::Sim3Transformation & out_transormation) const = 0;
  virtual void FindMatchingMapPoints(const KeyFrame * other,
                                     MapPointMatches & out_matches) const = 0;

  /*!
   *
   * @param map_points Map points in a different coordinate frame
   * @param relative_transformation
   * @param mp_local_transformation
   * @param out_visibles
   * @param radius_multiplier
   */
  virtual void FilterVisibleMapPoints(const MapPointSet &map_points,
                                      const geometry::Sim3Transformation &relative_transformation,
                                      const geometry::Pose &mp_local_transformation,
                                      precision_t radius_multiplier,
                                      std::list<MapPointVisibilityParams> &out_visibles) const = 0;

  virtual size_t AdjustSim3Transformation(std::list<MapPointVisibilityParams> & visibles,
                                          const KeyFrame * relative_kf,
                                          geometry::Sim3Transformation & in_out_transformation) const = 0;
  void ApplyStaging();

  virtual int GetScaleLevel(const map::MapPoint * map_point) const = 0;
  virtual int GetScaleLevel(const Observation & observation) const = 0;

  void SetStagingPosition(const geometry::Pose & pose) override;
  void SetStagingPosition(const g2o::SE3Quat & quaternion) override;
  void SetStagingPosition(const TMatrix33 & R, const TPoint3D & T) override;

 protected:
  void SerializeToStream(std::ostream & stream) const override;

 protected:
  virtual void InitializeImpl() = 0;
  virtual void AddMapPointImpl(Observation & observation) = 0;
  virtual void EraseMapPointImpl(Observation & observation) = 0;

 protected:
  bool is_initialized_;
  CovisibilityGraphNode covisibility_graph_;
  bool is_initial_;
  bool bad_flag_;

  // Container for saving the position after gba for LC
  geometry::Pose pose_gba_;
  KeyFrame * kf_gba_;
 public:
  std::vector<std::string> history;

};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_KEY_FRAME_H_
