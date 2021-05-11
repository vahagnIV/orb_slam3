//
// Created by vahagn on 19/04/2021.
//

#ifndef ORB_SLAM3_TEST_MOCK_MOCK_FEATURE_EXTRACTOR_H_
#define ORB_SLAM3_TEST_MOCK_MOCK_FEATURE_EXTRACTOR_H_
#include "../../src/features/ifeature_extractor.h"
namespace orb_slam3 {
namespace test {

class MockIFeatureExtractor : public IFeatureExtractor {
 public:
  MOCK_METHOD(int, Extract, (const TImageGray8U & image, Features & out_features), (override));
  MOCK_METHOD(precision_t, GetAcceptableSquareError, (unsigned level), (const, override));
  MOCK_METHOD(void, ComputeInvariantDistances, (const TPoint3D & point, const KeyPoint & key_point, precision_t & out_max_distance, precision_t & out_min_distance), (const, override));
  MOCK_METHOD(unsigned, PredictScale, (precision_t distance, precision_t max_distance), (const, override));
  MOCK_METHOD(unsigned, ComputeDistance, (const DescriptorType & d1, const DescriptorType & d2), (const, override));
  MOCK_METHOD(const std::vector<precision_t>&, GetScaleFactors, (), (const, override));
};

}  // namespace features
}  // namespace orb_slam3


#endif //ORB_SLAM3_TEST_MOCK_MOCK_FEATURE_EXTRACTOR_H_
