//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_RGBD_FRAME_H_
#define ORB_SLAM3_INCLUDE_RGBD_FRAME_H_

#include <vector>

#include <frame_base.h>
#include <typedefs.h>
#include <rgbd_camera.h>

namespace orb_slam3 {

class RGBDFrame : public FrameBase {
 public:
  RGBDFrame(const ImageRGB8U & image,
            const ImageGray32F & depth,
            double timestamp,
            const std::shared_ptr<RGBDCamera> & camera,
            const std::shared_ptr<IFeatureExtractor> & feature_extractor);
  int Compute() override;

 private:
  void InitializeMapPoints(const cv::Mat & points, const cv::Mat & undistorted_points);
  void AssignFeaturesToGrid();

 private:
  ImageGray8U gray_image_;
  ImageGray32F depth_image_;
  std::shared_ptr<RGBDCamera> camera_;


};

}

#endif //ORB_SLAM3_INCLUDE_RGBD_FRAME_H_
