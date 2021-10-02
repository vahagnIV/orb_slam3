//
// Created by vahagn on 19/04/2021.
//

#include "feature_tests.h"
#include <features/features.h>
#include "test_utils.h"

namespace orb_slam3 {
namespace test {

TEST_F(FeatureTests, ListFeaturesInAreaReturnsCorrectIndices) {
  auto camera = new camera::MonocularCamera(640, 480);
  camera->SetFx(800);
  camera->SetFy(800);
  camera->SetCx(320);
  camera->SetCy(240);
  camera->ComputeImageBounds();

  features::Features system_under_test(640,480);
  system_under_test.keypoints.emplace_back(TPoint2D{120, 62}, 0);
  system_under_test.keypoints.emplace_back(TPoint2D{20, 42}, 1);
  system_under_test.keypoints.emplace_back(TPoint2D{57, 259}, 0);
  system_under_test.keypoints.emplace_back(TPoint2D{185, 336}, 2);
  system_under_test.keypoints.emplace_back(TPoint2D{198, 78}, 0);
  system_under_test.keypoints.emplace_back(TPoint2D{1, 1}, 0);
  system_under_test.keypoints.emplace_back(TPoint2D{17, 451}, 5);
  system_under_test.keypoints.emplace_back(TPoint2D{301, 42}, 0);
  system_under_test.keypoints.emplace_back(TPoint2D{510, 79}, 1);
  system_under_test.keypoints.emplace_back(TPoint2D{604, 21}, 0);
  system_under_test.keypoints.emplace_back(TPoint2D{632, 262}, 0);

  system_under_test.undistorted_keypoints.resize(system_under_test.Size());
  system_under_test.undistorted_and_unprojected_keypoints.resize(system_under_test.Size());
  for (size_t i = 0; i < system_under_test.Size(); ++i) {
    camera->UndistortPoint(system_under_test.keypoints[i].pt, system_under_test.undistorted_keypoints[i]);
    camera->UnprojectAndUndistort(system_under_test.keypoints[i].pt, system_under_test.undistorted_and_unprojected_keypoints[i]);
  }

  system_under_test.AssignFeaturesToGrid();
  std::vector<std::size_t> result;
  system_under_test.ListFeaturesInArea(TPoint2D {190, 80}, 10, 0, 2, result);
  ASSERT_EQ(1, result.size());
  result.clear();
  system_under_test.ListFeaturesInArea(TPoint2D{190, 80}, 10, 1, 2, result);
  ASSERT_EQ(0, result.size());

  result.clear();
  system_under_test.ListFeaturesInArea(TPoint2D{190, 80}, 100, 0, 2, result);
  ASSERT_EQ(2, result.size());
  delete camera;
}

TEST_F(FeatureTests, FeatureDeserializationWorksCorrectly) {
  features::Features features(100, 200);
  features.descriptors.resize(100, 30);
  for (int i = 0; i < 100; ++i) {
    features::KeyPoint key_point(GenerateRandom2DPoint(0, 150, 0, 250), 1, 23, 5);
    features.keypoints.push_back(key_point);

    features.undistorted_keypoints.push_back(GenerateRandom2DPoint(0, 150, 0, 250));

  }
}

}
}