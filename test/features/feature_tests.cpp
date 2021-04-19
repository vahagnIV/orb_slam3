//
// Created by vahagn on 19/04/2021.
//

#include "feature_tests.h"
#include <features/features.h>
namespace orb_slam3 {
namespace test {

TEST_F(FeatureTests, ListFeaturesInAreaReturnsCorrectIndices){
  features::Features system_under_test(640, 480);
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

  system_under_test.AssignFeaturesToGrid();
  std::vector<std::size_t> result;
  system_under_test.ListFeaturesInArea(190, 80, 10, 0, 2, result);
  ASSERT_EQ(1, result.size());
  result.clear();
  system_under_test.ListFeaturesInArea(190, 80, 10, 1, 2, result);
  ASSERT_EQ(0, result.size());

  result.clear();
  system_under_test.ListFeaturesInArea(190, 80, 100, 0, 2, result);
  ASSERT_EQ(2, result.size());
}

}
}