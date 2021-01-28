//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_ORB_FEATURE_EXTRACTOR_H_
#define ORB_SLAM3_INCLUDE_ORB_FEATURE_EXTRACTOR_H_

#include <feature_extraction/ifeature_extractor.h>

namespace orb_slam3 {
namespace feature_extraction {

#define HALF_PATCH_SIZE 15
#define PATCH_SIZE 31
#define EDGE_THRESHOLD 19

class ORBFeatureExtractor : public IFeatureExtractor {
 public:
  ORBFeatureExtractor(unsigned image_width, unsigned image_height,
                      size_t features, precision_t scale_factor, size_t levels,
                      unsigned init_threshold_FAST,
                      unsigned min_threshold_FAST);
  int Extract(const TImageGray8U& image, TKeyPoints& out_keypoints,
              DescriptorSet& out_descriptors) override;

 private:
  void AllocatePyramid();
  void BuildImagePyramid(const TImageGray8U& image);
  template <typename TI, typename TO>
  void ResizeImage(
      const Eigen::Matrix<TI, Eigen::Dynamic,Eigen::Dynamic>& in,
      Eigen::Matrix<TO, Eigen::Dynamic, Eigen::Dynamic>& out_resized,
      size_t in_edge_left, 
      size_t in_edge_right, 
      size_t in_edge_top, 
      size_t in_edge_bottom,
      size_t out_edge_left, 
      size_t out_edge_right, 
      size_t out_edge_top, 
      size_t out_edge_bottom);

 private:
  unsigned image_width_;
  unsigned image_height_;

  unsigned features_;
  precision_t scale_factor_;
  unsigned init_threshold_FAST_;
  unsigned min_threshold_FAST_;

  std::vector<precision_t> scale_factors_;
  std::vector<precision_t> inv_scale_factors_;

  std::vector<precision_t> level_sigma2_;
  std::vector<precision_t> inv_level_sigma2_;

  std::vector<TImageGray> image_pyramid_;
  std::vector<int> features_per_level_;

  const static Eigen::Matrix<int, 256 * 2, 2> pattern_;

  const static int umax_[HALF_PATCH_SIZE + 1];
};

}  // namespace feature_extraction
}  // namespace orb_slam3
#endif  // ORB_SLAM3_INCLUDE_ORB_FEATURE_EXTRACTOR_H_
