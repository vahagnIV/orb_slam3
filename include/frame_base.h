//
// Created by vahagn on 11/29/20.
//

#ifndef ORB_SLAM3_INCLUDE_FRAME_BASE_H_
#define ORB_SLAM3_INCLUDE_FRAME_BASE_H_

#include <memory>

#include <opencv2/opencv.hpp>

#include <typedefs.h>
#include <ifeature_extractor.h>
#include <map_point.h>

namespace nvision {

class FrameBase {
 public:
  FrameBase() = default;
  FrameBase(double timestamp, const std::shared_ptr<IFeatureExtractor> & feature_extractor)
      : timestamp_(timestamp), feature_extractor_(feature_extractor) {}

  /*!
   * Getter function
   * @return the value of timestamp associated with the frame instance
   */
  inline double TimeStamp() const noexcept { return timestamp_; }

  /*!
   * Set the value of timestamp of the current frame
   * @param timestamp
   */
  inline void SetTimeStamp(double timestamp) noexcept { timestamp_ = timestamp; }

  /*!
   * This functions should be called after the constructor   *
   * @return 0 on success, positive on error
   */
  virtual int Compute() = 0;

  /*!
   * Initializes the position of the frame to identity, i.e. the frame is in the origin
   */
  void InitializeIdentity() noexcept;

  size_t FeatureCount() const noexcept { return key_points_.size(); }

  /*!
   * Destructor
   */
  virtual ~FrameBase() = default;
 protected:
  double timestamp_;
  const std::shared_ptr<IFeatureExtractor> feature_extractor_;
  std::vector<KeyPoint> key_points_;
  std::vector<MapPoint> map_points_;
  DescriptorSet descriptors_;
  cv::Matx44f current_pose_;

};

}

#endif //ORB_SLAM3_INCLUDE_FRAME_BASE_H_
