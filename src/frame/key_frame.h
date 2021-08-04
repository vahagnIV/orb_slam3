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

  KeyFrame(TimePoint time_point,
           const std::string & filename,
           const SensorConstants * sensor_constants,
           size_t id,
           std::shared_ptr<const features::handlers::BaseFeatureHandler> feature_handler);

  virtual ~KeyFrame() = default;

 public:

  virtual void Initialize() = 0;

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
  CovisibilityGraphNode & GetCovisibilityGraph() ;

  /*!
   * Const getter for the covisibility graph
   * @return
   */
  const CovisibilityGraphNode & GetCovisibilityGraph() const ;

  /*!
   * Checks if the frame is the initial frame in the map
   * @return true is initial
   */
  bool IsInitial() const ;

  /*!
   * Sets the initial flag to true
   * @param initial
   */
  void SetInitial(bool initial) ;

  /*!
   * Checks if the keyframe should be considered bad
   * @return true if bad
   */
  bool IsBad() const ;

  /*!
   * Sets the bad flag and removes the keyframe from the graph
   */
  virtual void SetBad();

  void SetPoseGBA(const TMatrix33 & R, const TVector3D & T);
  void SetKeyFrameGBA(KeyFrame * keyframe) { kf_gba_ = keyframe; }

  /*!
   * Creates new map points by matching features between the current frame and the "other"
   * @param other The other frame
   * @param out_newly_created output set of newly created map points
   */
  virtual void CreateNewMapPoints(frame::KeyFrame * other, MapPointSet & out_newly_created) = 0;

  /*!
   * Matches the map_points to the map points in the current keyframe and
   * replaces those that are matched with the corresponding map points of the current frame
   * @param map_points - the map points to match
   */
  virtual void FuseMapPoints(MapPointSet & map_points, bool use_staging) = 0;

  /*!
   * Erases the map point from the current frame
   */
  virtual void EraseMapPoint(const map::MapPoint *) = 0;

  /*!
   * Replaces the map point with the map point from the observation
   * @param map_point map point to replace
   * @param observation the new observation
   */
  virtual void ReplaceMapPoint(map::MapPoint * map_point, const Observation & observation) = 0;

  virtual bool FindSim3Transformation(const MapPointMatches & map_point_matches,
                                      const KeyFrame * other,
                                      geometry::Sim3Transformation & out_transormation) const = 0;
  virtual void FindMatchingMapPoints(const KeyFrame * other,
                                     MapPointMatches & out_matches) const = 0;

  virtual void FilterVisibleMapPoints(const MapPointSet & map_points,
                                      const geometry::Sim3Transformation & relative_transformation,
                                      const geometry::Pose & mp_local_transformation,
                                      std::list<MapPointVisibilityParams> & out_visibles,
                                      precision_t radius_multiplier) const = 0;

  virtual size_t AdjustSim3Transformation(std::list<MapPointVisibilityParams> & visibles,
                                          const KeyFrame * relative_kf,
                                          geometry::Sim3Transformation & in_out_transformation) const = 0;

 protected:
  CovisibilityGraphNode covisibility_graph_;
  bool is_initial_;
  bool bad_flag_;

  // Container for saving the position after gba for LC
  geometry::Pose pose_gba_;
  KeyFrame * kf_gba_;

};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_KEY_FRAME_H_
