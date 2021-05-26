//
// Created by vahagn on 31/03/2021.
//

#include "../../src/geometry/essential_matrix_estimator.h"
#include "../../src/features/features.h"

#include "essential_estimator_tests.h"
#include "test_utils.h"
#include <random>

namespace orb_slam3 {
namespace test {

const size_t NUMBER_OF_KEYPOINTS = 200;
const unsigned APPROX_NUMBER_OF_MATCHES = 150;

EssentialEstimatorTests::EssentialEstimatorTests()
    : camera_(InitializeCamera()), from_features_(camera_), to_features_(camera_) {
  GenerateRandomKeyPoints(from_features_.keypoints, ground_truth_points_, NUMBER_OF_KEYPOINTS, camera_);
  transformation_.R = GetRotationMatrixRollPitchYaw(M_PI / 180 * 10, M_PI / 180 * 4, M_PI / 180 * 5);
  transformation_.T = TVector3D{0.7, 1.2, 0.7};
  for (size_t i = 0; i < from_features_.Size(); ++i) {
    TPoint2D to_point;
    camera_->ProjectAndDistort(transformation_.Transform(ground_truth_points_[i]), to_point);
    to_features_.keypoints.emplace_back(to_point);
  }
  from_features_.UndistortKeyPoints();
  to_features_.UndistortKeyPoints();
  ground_truth_E_ = GetEssentialMatrixFromPose(transformation_);
}

void EssentialEstimatorTests::GenerateRandomKeyPoints(vector<features::KeyPoint> & out_keyoints,
                                                      std::vector<TPoint3D> & out_ground_truth_points,
                                                      size_t count,
                                                      const camera::MonocularCamera * camera) {

  geometry::Pose transformation;
  transformation.R = GetRotationMatrixRollPitchYaw(M_PI / 180 * 0.05, M_PI / 180 * 0.05, M_PI / 180 * 0.05);
  transformation.T = TVector3D{0.07, 0.02, 0.07};
  TMatrix33 E1 = GetEssentialMatrixFromPose(transformation);
  E1 = camera->K().inverse().transpose() * E1 * camera->K().inverse();

  transformation.R = GetRotationMatrixRollPitchYaw(M_PI / 180 * 10, M_PI / 180 * 14, M_PI / 180 * 5);
  transformation.T = TVector3D{0.6, 1.2, 0.7};
  TMatrix33 E2 = GetEssentialMatrixFromPose(transformation);
  E2 = camera->K().inverse().transpose() * E2 * camera->K().inverse();
  std::cout << "E1" << E1 / E1.norm() << std::endl;
  std::cout << "E2" << E2 / E2.norm() << std::endl;
  std::cout << " =========== " << std::endl;

  while (count--) {
    out_keyoints.emplace_back(GenerateRandom2DPoint(camera->ImageBoundMinX(),
                                                    camera->ImageBoundMinY(),
                                                    camera->ImageBoundMaxX(),
                                                    camera->ImageBoundMaxX()));
    TPoint3D unprojected;
    camera->UnprojectAndUndistort(out_keyoints.back().pt, unprojected);
    out_ground_truth_points.push_back(unprojected * DoubleRand(15, 42));
  }
}

camera::MonocularCamera * EssentialEstimatorTests::InitializeCamera() {
  const unsigned image_width = 640;
  const unsigned image_height = 480;
  auto camera = new camera::MonocularCamera(image_width, image_height);
  camera->SetFx(800);
  camera->SetCx(image_width / 2);
  camera->SetFy(800);
  camera->SetCy(image_height / 2);
  camera->ComputeImageBounds();
  return camera;
}

EssentialEstimatorTests::~EssentialEstimatorTests() {
  delete camera_;
}

TPoint2D EssentialEstimatorTests::GenerateGaussianNoise(precision_t sigma) {
  std::normal_distribution<precision_t> dist(0, sigma);
  std::random_device rd{};
  std::mt19937 gen{rd()};

  return orb_slam3::TPoint2D{dist(gen), dist(gen)};
}

TEST_F(EssentialEstimatorTests, EssentialMatrixCorrectlyRecoveredWithNoise) {
  precision_t sigma = 0.5;
  for (size_t i = 0; i < to_features_.Size(); ++i) {
    to_features_.keypoints[i].pt += GenerateGaussianNoise(sigma);
    from_features_.keypoints[i].pt += GenerateGaussianNoise(sigma);
  }
  to_features_.UndistortKeyPoints();
  from_features_.UndistortKeyPoints();

  std::unordered_map<std::size_t, std::size_t> matches;
  for (size_t i = 0, j = 0; i < to_features_.Size(); ++i) {
    if (rand() % to_features_.Size() > APPROX_NUMBER_OF_MATCHES)
      continue;
    matches[i] = i;
  }

  geometry::EssentialMatrixEstimator system_under_test(1);
  TMatrix33 E;

  std::unordered_set<std::size_t> inliers;
  precision_t score;
  std::vector<std::vector<std::size_t>> random_matches;

  GenerateRandomSubsets(0, matches.size(), 8, 200, matches, random_matches);

  system_under_test.FindBestEssentialMatrix(to_features_.undistorted_and_unprojected_keypoints,
                                            from_features_.undistorted_and_unprojected_keypoints,
                                            matches,
                                            random_matches,
                                            E,
                                            inliers,
                                            score, camera_);

  precision_t s = E(0, 0) * ground_truth_E_(0, 0) < 0 ? -1 : 1;

  ASSERT_LE((s * E - ground_truth_E_).norm(), 0.5);

  std::unordered_set<std::size_t> inliers3d;
  std::unordered_map<std::size_t, TPoint3D> triangulated;
  geometry::Pose estimated_pose;
  system_under_test.FindPose(E,
                             to_features_.undistorted_and_unprojected_keypoints,
                             from_features_.undistorted_and_unprojected_keypoints,
                             matches,
                             triangulated,
                             estimated_pose);

  ASSERT_LE((transformation_.R.transpose() * estimated_pose.R - TMatrix33::Identity(3, 3)).norm(), 0.5);

  estimated_pose.T *= transformation_.T.norm() / estimated_pose.T.norm();
  ASSERT_LE((estimated_pose.T - transformation_.T).norm(), 0.5);

  for (auto match: matches) {
    triangulated[match.first] *= ground_truth_points_[match.second].norm() / triangulated[match.first].norm();
    ASSERT_TRUE((triangulated[match.first] - ground_truth_points_[match.second]).norm() < 0.5);
  }
}

TEST_F(EssentialEstimatorTests, EssentialMatrixCorrectlyRecovered) {

  std::unordered_map<std::size_t, std::size_t> matches;
  for (size_t i = 0, j = 0; i < to_features_.Size(); ++i) {
    if (rand() % to_features_.Size() > APPROX_NUMBER_OF_MATCHES)
      continue;
    matches[i] = i;
  }

  geometry::EssentialMatrixEstimator system_under_test(1e-7);
  TMatrix33 E;

  std::unordered_set<std::size_t> inliers;
  precision_t score;
  std::vector<std::vector<std::size_t>> random_matches;

  GenerateRandomSubsets(0, matches.size(), 8, 5, matches, random_matches);

  system_under_test.FindBestEssentialMatrix(to_features_.undistorted_and_unprojected_keypoints,
                                            from_features_.undistorted_and_unprojected_keypoints,
                                            matches,
                                            random_matches,
                                            E,
                                            inliers,
                                            score, camera_);

  precision_t s = E(0, 0) * ground_truth_E_(0, 0) < 0 ? -1 : 1;

  ASSERT_LE((s * E - ground_truth_E_).norm(), 1e-11);

  std::unordered_set<std::size_t> inliers3d;
  std::unordered_map<std::size_t, TPoint3D> triangulated;
  geometry::Pose estimated_pose;
  system_under_test.FindPose(E,
                             to_features_.undistorted_and_unprojected_keypoints,
                             from_features_.undistorted_and_unprojected_keypoints,
                             matches,
                             triangulated,
                             estimated_pose);
  ASSERT_LE((transformation_.R - estimated_pose.R).norm(), 1e-12);

  estimated_pose.T *= transformation_.T.norm() / estimated_pose.T.norm();
  ASSERT_LE((estimated_pose.T - transformation_.T).norm(), 1e-11);

  for (auto match: matches) {
    triangulated[match.first] *= ground_truth_points_[match.second].norm() / triangulated[match.first].norm();
    ASSERT_TRUE((triangulated[match.first] - ground_truth_points_[match.second]).norm() < 1e-11);
  }

}

}
}
