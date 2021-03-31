//
// Created by vahagn on 31/03/2021.
//

#ifndef ORB_SLAM3_TEST_GEOMETRY_HOMOGRAPHY_ESTIMATOR_TESTS_H_
#define ORB_SLAM3_TEST_GEOMETRY_HOMOGRAPHY_ESTIMATOR_TESTS_H_

#include <gtest/gtest.h>
#include <typedefs.h>
namespace orb_slam3 {
namespace test {

class HomographyEstimatorTests : public ::testing::Test {
 public:
  HomographyEstimatorTests() = default;
  double CompareRatio(const TPoint3D & pt1, TPoint3D & pt2);

};

}
}

#endif //ORB_SLAM3_TEST_GEOMETRY_HOMOGRAPHY_ESTIMATOR_TESTS_H_
