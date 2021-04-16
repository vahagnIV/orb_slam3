//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_I_FEATURE_EXTRACTOR_H_
#define ORB_SLAM3_INCLUDE_I_FEATURE_EXTRACTOR_H_

// == stl ===
#include <vector>

// == orb-slam3 ===
#include <typedefs.h>
#include "features.h"
#include <features/key_point.h>

namespace orb_slam3 {
namespace features {

class IFeatureExtractor {
 public:
  /*!
   * Computes the keypoints and descriptor vectors from an image
   * @param image The image for which the key-points and descriptors are extracted
   * @param out_keypoints Key points
   * @param out_descriptors Descriptors
   * @return The number of extracted keypoints on success, -1 on fail.
   */
  virtual int Extract(const TImageGray8U & image,
                      Features & out_features) = 0;

  virtual precision_t GetAcceptableSquareError(unsigned level) const = 0;

  virtual void ComputeInvariantDistances(const TPoint3D & point,
                                         const KeyPoint & key_point,
                                         precision_t & out_max_distance,
                                         precision_t & out_min_distance) const = 0;

  virtual unsigned PredictScale(precision_t distance, precision_t max_distance) const = 0;

  virtual unsigned ComputeDistance(const DescriptorType & d1, const DescriptorType & d2) const = 0;
  virtual const std::vector<precision_t> & GetScaleFactors() const = 0;


  /*!
   * Virtual destructor
   */
  virtual ~IFeatureExtractor() = default;

};

}
}
#endif //ORB_SLAM3_INCLUDE_I_FEATURE_EXTRACTOR_H_
