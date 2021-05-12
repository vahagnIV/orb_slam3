//
// Created by vahagn on 19/04/2021.
//

#ifndef ORB_SLAM3_TEST_MOCK_MOCK_FRAME_BASE_H_
#define ORB_SLAM3_TEST_MOCK_MOCK_FRAME_BASE_H_

#include "../../src/frame/frame_base.h"
#include <gmock/gmock.h>

namespace orb_slam3 {
namespace test {

 class MockFrameBase : public frame::FrameBase {
 public:
   MockFrameBase(const TimePoint & timestamp,
                 const std::shared_ptr<features::IFeatureExtractor> & feature_extractor,
                 const std::string & filename): frame::FrameBase(timestamp, feature_extractor, filename) {  }
   MOCK_METHOD(orb_slam3::frame::FrameType, Type, (), (const, override));
   MOCK_METHOD(bool, IsValid, (), (const, override));
   MOCK_METHOD(bool, Link, (FrameBase * other), (override));
   MOCK_METHOD(void, ListMapPoints, (std::unordered_set<map::MapPoint *> & out_map_points), (const, override));
   MOCK_METHOD(TVector3D, GetNormal, (const TPoint3D & point), (const, override));
   MOCK_METHOD(bool, TrackWithReferenceKeyFrame, (FrameBase * reference_keyframe), (override));
   MOCK_METHOD(precision_t, ComputeMedianDepth, (), (const, override));
   MOCK_METHOD(void, SearchLocalPoints, (std::unordered_set<map::MapPoint *> & map_points), (override));


};

}  // namespace frame
}  // namespace orb_slam3

#endif //ORB_SLAM3_TEST_MOCK_MOCK_FRAME_BASE_H_
