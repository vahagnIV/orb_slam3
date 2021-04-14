//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_ORB_FEATURE_EXTRACTOR_H_
#define ORB_SLAM3_INCLUDE_ORB_FEATURE_EXTRACTOR_H_

// === opencv ===
#include <opencv2/opencv.hpp>

// == orb-slam3 ===
#include <features/ifeature_extractor.h>

namespace orb_slam3 {
namespace features {

#define HALF_PATCH_SIZE 15
#define PATCH_SIZE 31
#define EDGE_THRESHOLD 19

class ORBFeatureExtractor : public IFeatureExtractor {
 public:
  ORBFeatureExtractor(unsigned image_width, unsigned image_height,
                      size_t features, precision_t scale_factor, size_t levels,
                      unsigned init_threshold_FAST,
                      unsigned min_threshold_FAST);
  int Extract(const TImageGray8U & image, Features & out_features) override;
  precision_t GetAcceptableSquareError(unsigned int level) const override;
  void ComputeInvariantDistances(const TPoint3D & point,
                                 const KeyPoint & key_point,
                                 precision_t & out_max_distance,
                                 precision_t & out_min_distance) const override;
  unsigned int PredictScale(precision_t distance, precision_t max_distance) const override;
  unsigned int ComputeDistance(const DescriptorType & d1, const DescriptorType & d2) const override;
  const std::vector<precision_t> & GetScaleFactors() const override { return scale_factors_; }

 private:
  class ExtractorNode {
   public:
    ExtractorNode() : bNoMore(false) {}

    void DivideNode(ExtractorNode & n1, ExtractorNode & n2, ExtractorNode & n3,
                    ExtractorNode & n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
  };

 private:
  void AllocatePyramid();
  void BuildImagePyramid(cv::Mat & image);
  void ComputeKeyPointsOctTree(std::vector<std::vector<features::KeyPoint>> & out_all_keypoints);
  void DistributeOctTree(const std::vector<cv::KeyPoint> & vToDistributeKeys,
                         const int & minX,
                         const int & maxX,
                         const int & minY,
                         const int & maxY,
                         const int & nFeatures,
                         const int & level,
                         std::vector<features::KeyPoint> & out_map_points);
  static void computeOrientation(const cv::Mat & image,
                                 std::vector<features::KeyPoint> & keypoints,
                                 const int *umax);
  static void IC_Angle(const cv::Mat & image, features::KeyPoint &, const int *u_max);
  static void computeDescriptors(const cv::Mat & image,
                                 std::vector<features::KeyPoint> & keypoints,
                                 cv::Mat & descriptors,
                                 const std::vector<cv::Point> & pattern);
  static void computeOrbDescriptor(const features::KeyPoint & kpt, const cv::Mat & img,
                                   const cv::Point *pattern, uchar *desc);

 private :
  unsigned image_width_;
  unsigned image_height_;

  unsigned features_;
  precision_t scale_factor_;
  precision_t log_scale_factor_;
  unsigned init_threshold_FAST_;
  unsigned min_threshold_FAST_;

  std::vector<precision_t> scale_factors_;
  std::vector<precision_t> inv_scale_factors_;

  std::vector<precision_t> level_sigma2_;
  std::vector<precision_t> inv_level_sigma2_;

  std::vector<cv::Mat> image_pyramid_;
  // std::vector<TImageGray> image_pyramid_;
  std::vector<int> features_per_level_;

  // const static Eigen::Matrix<int, 256 * 2, 2> pattern_;
  const static std::vector<cv::Point> pattern_;

  const static int umax_[HALF_PATCH_SIZE + 1];

  float lapping_area_start_, lapping_area_end_;
};

}  // namespace features
}  // namespace orb_slam3
#endif  // ORB_SLAM3_INCLUDE_ORB_FEATURE_EXTRACTOR_H_
