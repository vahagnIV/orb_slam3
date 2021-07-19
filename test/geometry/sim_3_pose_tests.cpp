//
// Created by vahagn on 19/07/2021.
//

#include "sim_3_pose_tests.h"
#include "test_utils.h"
#include <geometry/sim3_transformation.h>

namespace orb_slam3 {
namespace test {

TEST_F(Sim3PoseTests, InverseReturnsCorrectTransformation) {
  geometry::Sim3Transformation system_under_test;
  system_under_test.R = GetRotationMatrixRollPitchYaw(0.2, 1.5, 0.7);
  system_under_test.T << 1, 6, 2;
  system_under_test.s = 2.2;

  geometry::Sim3Transformation inverse = system_under_test.GetInversePose();
  geometry::Sim3Transformation mul = inverse * system_under_test;
  ASSERT_DOUBLE_EQ(mul.s, 1);
  ASSERT_LE((mul.R - TMatrix33::Identity()).norm(), 1e-15);
  ASSERT_LE(mul.T.norm(), 1e-15);
}

}
}