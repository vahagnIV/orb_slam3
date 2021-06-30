//
// Created by vahagn on 22/06/2021.
//

#include "sim3_solver_tests.h"
#include <test_utils.h>
#include <geometry/sim3_solver.h>
namespace orb_slam3 {
namespace test {

TEST_F(Sim3SolverTests, TransformationCorrectlyRecovered) {
  geometry::Pose gt_pose;
  gt_pose.R = GetRotationMatrixRollPitchYaw(0.2, 0.1, 0.05);
  gt_pose.T = TVector3D{1.7, 2.6, 1.5};
  gt_pose.s = 1.75;

  std::vector<std::pair<TPoint3D, TPoint3D>> matches(20);
  std::vector<size_t> slice_indices;
  for (int i = 0; i < 20; ++i) {
    matches[i].second = GenerateRandomHomogenousPoint(5, 10) * DoubleRand(0.5, 6);
    matches[i].first = gt_pose.Transform(matches[i].second);

  }
  slice_indices = {1, 5, 9, 12, 19};

  geometry::Pose res = geometry::Sim3Solver::ComputeSim3(matches, slice_indices);

  ASSERT_LE((res.R - gt_pose.R).norm(), 1e-7);
  ASSERT_DOUBLE_EQ(gt_pose.s, res.s);
  ASSERT_LE((res.T - gt_pose.T).norm(), 1e-7);

}

}
}