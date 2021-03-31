//
// Created by vahagn on 31/03/2021.
//

#include <geometry/essential_matrix_estimator.h>

#include "essential_estimator_tests.h"
#include "test_utils.h"

namespace orb_slam3 {
namespace test {

TEST_F(EssentialEstimatorTests, EssentialMatrixCorrectlyRecovered) {
  std::vector<TPoint3D> points_from(8), points_to(8);
  points_from[0] << 0.5, 2.6, 8.1;
  points_from[1] << 0.3, 3.6, 9.6;
  points_from[2] << 0.9, -4.2, 4.3;
  points_from[3] << -1.5, -3.1, 2.8;
  points_from[4] << 2.5, 2.9, 5.1;
  points_from[5] << 1.99, 3.1, 6.3;
  points_from[6] << 2.5, 1.6, 2.7;
  points_from[7] << -0.5, 0.6, 11.4;

  TMatrix33 R = GetRotationMatrixRollPitchYaw(0, 0, M_PI / 6);;

  TVector3D T;
  T << 1.2, -7.5, 4.1;

  std::transform(points_from.begin(),
                 points_from.end(),
                 points_to.begin(),
                 [&](const TPoint3D & pt) { return R * pt + T; });
  geometry::EssentialMatrixEstimator system_under_test(1e-7);
  std::vector<features::Match> matches;
  std::vector<size_t> random_subset_idx(8);
  for (size_t i = 0; i < points_from.size(); ++i) {
    matches.emplace_back(i, i);
    random_subset_idx[i] = i;
  }
  TMatrix33 E;
  system_under_test.FindEssentialMatrix(points_to, points_from, matches, random_subset_idx, E);
  for (size_t i = 0; i < points_from.size(); ++i) {
    ASSERT_TRUE(std::abs(points_to[i].dot(E * points_from[i])) < 1e-12);
  }

  ASSERT_TRUE(E.determinant() < 1e-6);

  std::vector<bool> inliers;
  std::vector<TPoint3D> triangulated;
  geometry::Pose pose;
  for (int i = 0; i < points_to.size(); ++i) {
    points_to[i] /= points_to[i][2];
    points_from[i] /= points_from[i][2];
  }
  system_under_test.FindPose(E, points_to, points_from, matches, inliers, triangulated, pose);
  ASSERT_TRUE((pose.T.normalized() - T.normalized()).norm() < 1e-12);
  ASSERT_TRUE((pose.R - R).norm() < 1e-12);

}

}
}