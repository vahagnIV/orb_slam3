//
// Created by vahagn on 31/03/2021.
//

#include <geometry/essential_matrix_estimator.h>
#include <features/features.h>

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
//  for (int i = 0; i < points_from.size(); ++i) {
//    points_to[points_to.size() - i - 1] = R * points_from[i] + T;
//  }



//  geometry::EssentialMatrixEstimator system_under_test(1e-7);
//  std::unordered_map<std::size_t, std::size_t> matches;
//  std::vector<size_t> random_subset_idx(8);
//  for (size_t i = 0; i < points_from.size(); ++i) {
//    matches.emplace(i, i);
//    random_subset_idx[i] = i;
//  }
//  TMatrix33 E;
//  system_under_test.FindEssentialMatrix(points_to, points_from, matches, random_subset_idx, E);
//  for (size_t i = 0; i < points_from.size(); ++i) {
//    ASSERT_TRUE(std::abs(points_to[i].dot(E * points_from[i])) < 1e-12);
//  }
//
//  ASSERT_TRUE(E.determinant() < 1e-6);
//
//  std::unordered_set<std::size_t> inliers;
//  std::unordered_map<size_t, TPoint3D> triangulated;
//  geometry::Pose pose;
//  for (int i = 0; i < points_to.size(); ++i) {
//    points_to[i] /= points_to[i][2];
//    points_from[i] /= points_from[i][2];
//  }
//  system_under_test.FindPose(E, points_to, points_from, matches, inliers, triangulated, pose);
//  ASSERT_TRUE((pose.T.normalized() - T.normalized()).norm() < 1e-12);
//  ASSERT_TRUE((pose.R - R).norm() < 1e-12);

}

TEST_F(EssentialEstimatorTests, AAA) {
  const unsigned image_width = 640;
  const unsigned image_height = 480;
  const size_t keypoint_count = 200;
  const unsigned match_count = 150;

  features::Features imfrom_features(image_width, image_height), imto_features(image_width, image_height);
  for (size_t i = 0; i < keypoint_count; ++i) {
    imfrom_features.keypoints.emplace_back(GenerateRandom2DPoint(image_width, image_height));
  }

  precision_t fx = 800;
  precision_t fy = 800;
  precision_t cx = image_width / 2;
  precision_t cy = image_height / 2;

  std::vector<TPoint3D> gt_points;

  for (features::KeyPoint & kp: imfrom_features.keypoints) {
    HomogenousPoint undistorted_point{(kp.pt.x() - cx) / fx, (kp.pt.y() - cy) / fy, 1};
    imfrom_features.undistorted_and_unprojected_keypoints.push_back(undistorted_point);
    gt_points.push_back(undistorted_point * DoubleRand(5, 45));
  }

  geometry::Pose pose;
  pose.R = GetRotationMatrixRollPitchYaw(M_PI / 180 * 10, M_PI / 180 * 4, M_PI / 180 * 5);
  pose.T = TVector3D{0.7, 1.2, 0.7};

  std::unordered_map<std::size_t, std::size_t> matches;
  for (size_t i = 0; i < keypoint_count; ++i) {
    if (rand() % imfrom_features.Size() > match_count)
      continue;

    auto transformed_point = pose.Transform(gt_points[i]);
    features::KeyPoint & from_kp = imfrom_features.keypoints[i];
    features::KeyPoint to_kp(TPoint2D{from_kp.pt.x() * fx + cx, from_kp.pt.y() * fy + cy});
    imto_features.keypoints.push_back(to_kp);

    imto_features.undistorted_and_unprojected_keypoints.push_back(HomogenousPoint{
        transformed_point.x() / transformed_point.z(), transformed_point.y() / transformed_point.z(), 1});
    matches[imto_features.undistorted_and_unprojected_keypoints.size() - 1] = i;
  }

  geometry::EssentialMatrixEstimator system_under_test(1e-7);
  TMatrix33 E, gt_E = GetEssentialMatrixFromPose(pose);

  std::unordered_set<std::size_t> inliers;
  precision_t score;
  std::vector<std::vector<std::size_t>> random_matches;

  GenerateRandomSubsets(0, matches.size(), 8, 5, matches, random_matches);

  system_under_test.FindBestEssentialMatrix(imto_features.undistorted_and_unprojected_keypoints,
                                            imfrom_features.undistorted_and_unprojected_keypoints,
                                            matches,
                                            random_matches,
                                            E,
                                            inliers,
                                            score);


  precision_t s = E(0, 0) * gt_E(0, 0) < 0 ? -1 : 1;

  ASSERT_TRUE((s * E - gt_E).norm() < 1e-10);

  std::unordered_set<std::size_t> inliers3d;
  std::unordered_map<std::size_t, TPoint3D> triangulated;
  geometry::Pose estimated_pose;
  system_under_test.FindPose(E,
                             imto_features.undistorted_and_unprojected_keypoints,
                             imfrom_features.undistorted_and_unprojected_keypoints,
                             matches,
                             triangulated,
                             estimated_pose);

  ASSERT_TRUE((pose.R - estimated_pose.R).norm() < 1e-11);

  estimated_pose.T *= pose.T.norm() / estimated_pose.T.norm();
  ASSERT_TRUE((estimated_pose.T - pose.T).norm() < 1e-11);

  for (auto match: matches) {
    if (inliers3d.find(match.first) == inliers3d.end())
      continue;
    triangulated[match.first] *= gt_points[match.second].norm() / triangulated[match.first].norm();
    ASSERT_TRUE((triangulated[match.first] - gt_points[match.second]).norm() < 1e-11);
  }

}

}
}
