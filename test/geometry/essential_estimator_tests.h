//
// Created by vahagn on 31/03/2021.
//

#ifndef ORB_SLAM3_TEST_GEOMETRY_ESSENTIAL_ESTIMATOR_TESTS_H_
#define ORB_SLAM3_TEST_GEOMETRY_ESSENTIAL_ESTIMATOR_TESTS_H_
#include <gtest/gtest.h>

namespace orb_slam3 {
namespace test {

class EssentialEstimatorTests : public ::testing::Test {
 public:
  EssentialEstimatorTests();
  ~EssentialEstimatorTests() override;
 protected:
  static void GenerateRandomKeyPoints(std::vector<features::KeyPoint> & out_keyoints,
                                      std::vector<TPoint3D> & out_ground_truth_points,
                                      size_t count,
                                      const camera::MonocularCamera * camera);
  static TPoint2D GenerateGaussianNoise(precision_t sigma);
 protected:
  const camera::MonocularCamera * camera_;
  features::Features from_features_;
  features::Features to_features_;
  std::vector<TPoint3D> ground_truth_points_;
  TMatrix33 ground_truth_E_;
  geometry::Pose transformation_;

};

}
}
#endif //ORB_SLAM3_TEST_GEOMETRY_ESSENTIAL_ESTIMATOR_TESTS_H_
