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
  for (decltype(features.descriptors)::Index i = 0; i < features.descriptors.rows(); ++i) {
    for (decltype(features.descriptors)::Index j = 0; j < features.descriptors.cols(); ++j) {
      features.descriptors(i, j) = static_cast<unsigned char>(rand()  % 0xFF);
    }
  }
  for (int i = 0; i < 100; ++i) {
    features::KeyPoint key_point(GenerateRandom2DPoint(0, 0, 150, 250), 1, 23, 5);
    features.keypoints.push_back(key_point);

    features.undistorted_keypoints.push_back(GenerateRandom2DPoint(0, 0, 150, 250));
    features.undistorted_and_unprojected_keypoints.push_back(GenerateRandom3DPoint(0, 0, 0, 10, 20, 30));

  }
  std::stringstream stream(std::ios::binary | std::ios::in | std::ios::out);
  stream << features;

  stream.seekp(0);

  features::Features features2(stream);

  ASSERT_EQ(features.image_width, features2.image_width);
  ASSERT_EQ(features.image_height, features2.image_height);
  ASSERT_EQ(0, (features.descriptors - features2.descriptors).cwiseAbs().sum());
  ASSERT_EQ(features.keypoints.size(), features2.keypoints.size());
  for (size_t i = 0; i < features.keypoints.size(); ++i) {
    const features::KeyPoint &kp1 = features.keypoints[i];
    const features::KeyPoint &kp2 = features2.keypoints[i];
    ASSERT_EQ(kp1.angle, kp2.angle);
    ASSERT_EQ(kp1.size, kp2.size);
    ASSERT_EQ(kp1.level, kp2.level);
    ASSERT_EQ(kp1.X(), kp2.X());
    ASSERT_EQ(kp1.Y(), kp2.Y());
  }
  for (size_t i = 0; i < features.keypoints.size(); ++i) {
    const TPoint2D &ukp1 = features.undistorted_keypoints[i];
    const TPoint2D &ukp2 = features2.undistorted_keypoints[i];
    ASSERT_EQ(ukp1.x(), ukp2.x());
    ASSERT_EQ(ukp1.y(), ukp2.y());
  }

  for (size_t i = 0; i < features.undistorted_and_unprojected_keypoints.size(); ++i) {
    const TPoint3D &ukp1 = features.undistorted_and_unprojected_keypoints[i];
    const TPoint3D &ukp2 = features2.undistorted_and_unprojected_keypoints[i];
    ASSERT_EQ(ukp1.x(), ukp2.x());
    ASSERT_EQ(ukp1.y(), ukp2.y());
    ASSERT_EQ(ukp1.z(), ukp2.z());
  }
}

}
}