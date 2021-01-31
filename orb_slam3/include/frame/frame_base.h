//
// Created by vahagn on 11/29/20.
//

#ifndef ORB_SLAM3_INCLUDE_FRAME_BASE_H_
#define ORB_SLAM3_INCLUDE_FRAME_BASE_H_

/// stl
#include <memory>

/// orb_slam3
#include <typedefs.h>
#include <feature_extraction/ifeature_extractor.h>
#include <map/map_point.h>

namespace orb_slam3 {
namespace frame{

class FrameBase {
 public:
  typedef size_t id_type;
  FrameBase() = delete;

  FrameBase(const TimePoint & timestamp,
            const std::shared_ptr<feature_extraction::IFeatureExtractor> &feature_extractor)
      : id_(++next_id_),timestamp_(timestamp), feature_extractor_(feature_extractor) {}

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
  void SetPosition(const cv::Matx44f &position) noexcept;

  // Get the number of the extracted keypoints
  size_t FeatureCount() const noexcept { return map_points_.size(); }

  virtual bool IsValid() const = 0;


  /*!
   * Destructor
   */
  virtual ~FrameBase() = default;

 protected:



 protected:
  const id_type id_;
  TimePoint timestamp_;
  const std::shared_ptr<feature_extraction::IFeatureExtractor> feature_extractor_;
  std::vector<map::MapPoint> map_points_;

  protected:
    static id_type next_id_;



};

}
}

#endif //ORB_SLAM3_INCLUDE_FRAME_BASE_H_
