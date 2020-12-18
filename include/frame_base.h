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
  FrameBase() ;

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

  /*!
   * Set the reference frame for this frame
   * @param reference_frame
   */
  void SetReferenceFrame(FrameBase *reference_frame) noexcept { reference_frame_ = reference_frame; }

  /*!
   * Destructor
   */
  virtual ~FrameBase() = default;

 protected:
  static void CvKeypointsToMat(const std::vector<cv::KeyPoint> &keypoints, cv::Mat &out_points);

 protected:
  double timestamp_;
  const std::shared_ptr<IFeatureExtractor> feature_extractor_;
  std::vector<MapPoint> map_points_;
  DescriptorSet descriptors_;
  FrameBase *reference_frame_;

  T3DTransformationMatrix R_camera2world_;
  T3DTransformationMatrix R_world2camera_;
  T3DVector T_camera2world_;
  T3DVector T_0world_;

};

}

#endif //ORB_SLAM3_INCLUDE_FRAME_BASE_H_
