//
// Created by vahagn on 31/03/2021.
//

#include "homography_estimator_tests.h"
#include <geometry/homography_matrix_estimator.h>
namespace orb_slam3 {
namespace test {

double HomographyEstimatorTests::CompareRatio(const TPoint3D & pt1, TPoint3D & pt2) {
  double x = (pt1[0] / pt1[2] - pt2[0] / pt2[2]);
  double y = (pt1[1] / pt1[2] - pt2[1] / pt2[2]);
  double result = std::sqrt(x * x + y * y);
  return result;
}

TEST_F(HomographyEstimatorTests, HomographyEstimaterWorksCorrectly) {
  std::vector<TPoint3D> points_from(8), points_to(8);
  points_from[0] << 0.5, 2.6, 8.1;
  points_from[1] << 0.3, 3.6, 8.1;
  points_from[2] << 0.9, -4.2, 8.1;
  points_from[3] << -1.5, -3.1, 8.1;
  points_from[4] << 2.5, 2.9, 8.1;
  points_from[5] << 1.99, 3.1, 8.1;
  points_from[6] << 2.5, 1.6, 8.1;
  points_from[7] << -0.5, 0.6, 8.1;

  TMatrix33 R;
  R << 0.967303, 0.0209349, -0.252757,
      -0.0431297, 0.99565, -0.0825917,
      0.249928, 0.0907926, 0.963998;
  TVector3D T;
  T << 1.2, -7.5, 4.1;

  std::transform(points_from.begin(),
                 points_from.end(),
                 points_to.begin(),
                 [&](const TPoint3D & pt) { return R * pt + T; });
  std::vector<features::Match> matches;
  std::vector<size_t> random_subset_idx(8);
  for (size_t i = 0; i < points_from.size(); ++i) {
    matches.emplace_back(i, i);
    random_subset_idx[i] = i;
  }
  TMatrix33 H;
  geometry::HomographyMatrixEstimator estimator(1e-7);
  estimator.FindHomographyMatrix(points_to, points_from, matches, random_subset_idx, H);
  for (size_t i = 0; i < matches.size(); ++i) {
    ASSERT_TRUE(CompareRatio(H * points_from[i], points_to[i]) < 1e-12);
    ASSERT_TRUE(CompareRatio(H.inverse() * points_to[i], points_from[i]) < 1e-12);
  }

  std::vector<bool> inliers;
  std::vector<TPoint3D> triangulated;
  geometry::Pose pose;
  estimator.FindPose(H, points_to, points_from, matches, inliers, triangulated, pose);
  ASSERT_TRUE((pose.T.normalized() - T.normalized()).norm() < 1e-12);
  ASSERT_TRUE((pose.R - R).norm() < 1e-12);
}

}
}