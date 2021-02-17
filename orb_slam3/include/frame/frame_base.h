//
// Created by vahagn on 11/29/20.
//

#ifndef ORB_SLAM3_INCLUDE_FRAME_BASE_H_
#define ORB_SLAM3_INCLUDE_FRAME_BASE_H_

/// stl
#include <memory>

/// orb_slam3
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

  FrameBase(const TimePoint & timestamp,
            const std::shared_ptr<features::IFeatureExtractor> & feature_extractor)
      : id_(++next_id_), timestamp_(timestamp), feature_extractor_(feature_extractor), previous_frame_(nullptr) {}
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
   * @param position 4x4 joint transformation matrix
   */
  void SetPosition(const cv::Matx44f & position) noexcept;

  // Get the number of the extracted keypoints
  virtual size_t FeatureCount() const noexcept = 0;

  /*!
   * If frame passed certain checks like the number of features etc
   * @return true if valid
   */
  virtual bool IsValid() const = 0;

  /*!
   * Setter for the previous frame
   * @param previous_frame the previous frame
   */
  void SetPrevious(const std::shared_ptr<FrameBase> & previous_frame) { previous_frame_ = previous_frame; }

  /*!
   * Getter for the previous frame
   * @return the previous frame
   */
  const std::shared_ptr<FrameBase> & PreviousFrame() { return previous_frame_; }

  /*!
   * If needed, this should inizialize the position from the previous frames
   * Used only for monocular case
   * @return
   */
  virtual bool InitializePositionFromPrevious() = 0;

  /*!
   * Destructor
   */
  virtual ~FrameBase() = default;

 protected:

 protected:

  const id_type id_;
  TimePoint timestamp_;
  const std::shared_ptr<features::IFeatureExtractor> feature_extractor_;
  std::vector<map::MapPoint> map_points_;
  std::shared_ptr<FrameBase> previous_frame_;
  geometry::Pose pose_;
 protected:
  static id_type next_id_;

};

}
}

#endif //ORB_SLAM3_INCLUDE_FRAME_BASE_H_
