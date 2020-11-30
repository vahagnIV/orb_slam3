//
// Created by vahagn on 11/29/20.
//

#ifndef ORB_SLAM3_INCLUDE_FRAME_BASE_H_
#define ORB_SLAM3_INCLUDE_FRAME_BASE_H_

#include <typedefs.h>
#include <memory>
#include <ifeature_extractor.h>

namespace nvision {

class FrameBase {
 public:
  FrameBase() = default;
  FrameBase(double timestamp, const std::shared_ptr<IFeatureExtractor> &feature_extractor)
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
   * Destructor
   */
  virtual ~FrameBase() = default;
 protected:
  double timestamp_;
  const std::shared_ptr<IFeatureExtractor> feature_extractor_;

};

}

#endif //ORB_SLAM3_INCLUDE_FRAME_BASE_H_
