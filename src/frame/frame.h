//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_FRAME_H_
#include "base_frame.h"
#include <frame/map_point_visibility_params.h>

namespace orb_slam3 {

namespace frame {
class KeyFrame;

class Frame : public BaseFrame {

 public:
  Frame(TimePoint & time_point,
        const std::string & filename,
        const SensorConstants * sensor_constants) : BaseFrame(time_point,
                                                              filename,
                                                              sensor_constants, ++next_id_) {}

  virtual ~Frame() = default;
 public:

  /*!
   * Check if a frame is valid
   * @return true if valid
   */
  virtual bool IsValid() const = 0;

  /*!
   * Uses the "other" frame to estimate the relative position.  Creates Map Points and stores
   * them to the internal structures of both frames. This function is used for Monocular case only,
   * because only in this case the map points cannot be created immediately.
   * This is the only function of frame that creates map points
   * @param other
   * @return
   */
  virtual bool Link(Frame * other) = 0;

  /*!
   * Matches map_points from the reference keyframe using descriptor matching and stores
   * the found map points in the internal data structure.
   * If enough matches are found the optimization is performed.
   * @param reference_keyframe The reference key frame.
   * @return true if there are enough estimated map points and the optimization result is reliable.
   */
  virtual bool FindMapPointsFromReferenceKeyFrame(const KeyFrame * reference_keyframe) = 0;

  /*!
   * Estimates position by projecting map points from the argument frame.
   * The function assumes that a good initial guess is set for the frame's position.
   * @param frame The frame from which the map points are taken
   * @param out_visibles The list of visible map points
   * @return true if the position is reliably estimated
   */
  virtual bool EstimatePositionByProjectingMapPoints(Frame * frame,
                                                     std::list<MapPointVisibilityParams> & out_visibles) = 0;

  /*!
   * Creates a KeyFrame from the current frame.
   * @return The new keyframe
   */
  virtual KeyFrame * CreateKeyFrame() = 0;

  /*!
   * Optimizes the frame pose by minimizing the Map point projection error.
   */
  virtual void OptimizePose() = 0;

  /*!
   * Return the number of map points in the frame
   * @return The number of map points in the frame
   */
  virtual size_t GetMapPointCount() const = 0;

  /*!
   * Filters all map points that are visible in the current frame.
   * @param map_points The initial list of candidate map points
   * @param out_filtered_map_points The filtered map points wrapped in the structures that provide additional
   * information of the way the map point is visible
   * @param radius_multiplier The radius multiplier to search
   */
  virtual void FilterVisibleMapPoints(const std::unordered_set<map::MapPoint *> & map_points,
                                      std::list<MapPointVisibilityParams> & out_filtered_map_points,
                                      precision_t radius_multiplier) const = 0;

  /*!
   * Matches the visible map points to the local keypoints and populates the internal data structure.
   * @param filtered_map_points The candidate visible map points.
   */
  virtual void SearchInVisiblePoints(const std::list<MapPointVisibilityParams> & filtered_map_points) = 0;

  /*!
   * Updates the frame's position and map points from the reference keyframe. Usually called when the
   * corresponding keyframe is optimized.
   */
  virtual void UpdateFromReferenceKeyFrame() = 0;
 private:
  static size_t next_id_;

};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_FRAME_H_
