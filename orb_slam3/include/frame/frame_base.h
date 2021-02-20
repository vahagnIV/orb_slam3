//
// Created by vahagn on 11/29/20.
//

#ifndef ORB_SLAM3_INCLUDE_FRAME_BASE_H_
#define ORB_SLAM3_INCLUDE_FRAME_BASE_H_

// ===stl===
#include <memory>

// == orb-slam3 ===
#include "frame_type.h"
#include <typedefs.h>
#include <features/ifeature_extractor.h>
#include <geometry/pose.h>
#include <map/map_point.h>

namespace orb_slam3 {
namespace frame {

class FrameBase {
 public:
  typedef size_t id_type;
  FrameBase() = delete;

  virtual FrameType Type() const = 0;

  FrameBase(const TimePoint & timestamp)
      : id_(++next_id_), timestamp_(timestamp) {}
  /*!
  *  Getter for id
  *  @return The id of the frame
  */
  inline id_type Id() const noexcept { return id_; }

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

  // Get the number of the extracted keypoints
  virtual size_t FeatureCount() const noexcept = 0;

  /*!
   * If frame passed certain checks like the number of features etc
   * @return true if valid
   */
  virtual bool IsValid() const = 0;

  /*!
   * If needed, this should inizialize the position from the previous frames
   * Used only for monocular case
   * @return
   */
  virtual bool Link(const std::shared_ptr<FrameBase> & other) = 0;

  const std::shared_ptr<map::MapPoint> & MapPoint(size_t id) const { return map_points_[id]; }
  std::shared_ptr<map::MapPoint> & MapPoint(size_t id) { return map_points_[id]; }
  std::vector<std::shared_ptr<map::MapPoint>> & MapPoints() { return map_points_; }

  /*!
   * Destructor
   */
  virtual ~FrameBase() = default;

 protected:

 protected:

  const id_type id_;
  TimePoint timestamp_;
  std::vector<std::shared_ptr<map::MapPoint>> map_points_;
  geometry::Pose pose_;
 protected:
  static id_type next_id_;

};

}
}

#endif //ORB_SLAM3_INCLUDE_FRAME_BASE_H_
