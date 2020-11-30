//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_RGBD_FRAME_H_
#define ORB_SLAM3_INCLUDE_RGBD_FRAME_H_

#include <vector>

#include <frame_base.h>
#include <icamera.h>
#include <typedefs.h>

namespace nvision {

class RGBDFrame : public FrameBase {
 public:
  RGBDFrame(const ImageRGB8U &image,
            const ImageGray32F &depth,
            double timestamp,
            const std::shared_ptr<ICamera> & camera,
            const std::shared_ptr<IFeatureExtractor> &feature_extractor);
  int Compute() override;
 private:
  ImageGray8U gray_image_;
  std::vector<KeyPoint> key_points_;
  std::shared_ptr<ICamera> camera_;
  DescriptorSet descriptors_;

};

}

#endif //ORB_SLAM3_INCLUDE_RGBD_FRAME_H_
