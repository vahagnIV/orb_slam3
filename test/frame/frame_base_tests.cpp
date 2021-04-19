//
// Created by vahagn on 19/04/2021.
//

#include "frame_base_tests.h"
#include "mock/mock_frame_base.h"
#include "gtest/gtest.h"
#include <gmock/gmock.h>

namespace orb_slam3 {
namespace test {
using ::testing::_;
using ::testing::Invoke;
TEST_F(FrameBaseTests, LiastAllMapPointsReturnsallMapPoints) {
  MockFrameBase f1(std::chrono::system_clock::now(), nullptr, "f1"),
      f2(std::chrono::system_clock::now(), nullptr, "f2"),
      f3(std::chrono::system_clock::now(), nullptr, "f3");

  std::vector<map::MapPoint *> frame1_map_points, frame2_map_points, frame3_map_points;
  frame1_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));
  frame1_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));
  frame1_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));
  frame1_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));
  frame1_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));

  frame2_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));
  frame2_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));
  frame2_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));
  frame2_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));
  frame2_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));

  frame3_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));
  frame3_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));
  frame3_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));
  frame3_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));
  frame3_map_points.emplace_back(new map::MapPoint(TPoint3D{0, 0, 0}, 0, 0));

  auto lambda1 = [&frame1_map_points](std::unordered_set<map::MapPoint *> & out_mp) {
    std::copy(frame1_map_points.begin(),
              frame1_map_points.end(),
              std::inserter<std::unordered_set<map::MapPoint *>, std::unordered_set<map::MapPoint *>::iterator>(out_mp,
                                                                                                                out_mp.begin()));
  };
  auto lambda2 = [&frame2_map_points](std::unordered_set<map::MapPoint *> & out_mp) {
    std::copy(frame2_map_points.begin(),
              frame2_map_points.end(),
              std::inserter<std::unordered_set<map::MapPoint *>, std::unordered_set<map::MapPoint *>::iterator>(out_mp,
                                                                                                                out_mp.begin()));
  };
  auto lambda3 = [&frame3_map_points](std::unordered_set<map::MapPoint *> & out_mp) {
    std::copy(frame3_map_points.begin(),
              frame3_map_points.end(),
              std::inserter<std::unordered_set<map::MapPoint *>, std::unordered_set<map::MapPoint *>::iterator>(out_mp,
                                                                                                                out_mp.begin()));
  };

  EXPECT_CALL(f1, ListMapPoints(_)).WillOnce(Invoke(lambda1));
  EXPECT_CALL(f2, ListMapPoints(_)).WillOnce(Invoke(lambda2));
  EXPECT_CALL(f3, ListMapPoints(_)).WillOnce(Invoke(lambda3));

  MockFrameBase system_under_test(std::chrono::system_clock::now(), nullptr, "system_under_test");
  std::unordered_set<map::MapPoint *> map_points;
  std::unordered_set<frame::FrameBase *> frames{&f1, &f2, &f3};
  system_under_test.ListAllMapPoints(frames, map_points);
  ASSERT_EQ(map_points.size(), frame1_map_points.size() + frame2_map_points.size() + frame3_map_points.size());
}

}
}