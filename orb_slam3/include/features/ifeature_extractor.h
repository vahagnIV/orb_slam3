//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_I_FEATURE_EXTRACTOR_H_
#define ORB_SLAM3_INCLUDE_I_FEATURE_EXTRACTOR_H_

#include <vector>
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
                      Features & features) = 0;

  virtual ~IFeatureExtractor() = default;

};

}
}
#endif //ORB_SLAM3_INCLUDE_I_FEATURE_EXTRACTOR_H_
