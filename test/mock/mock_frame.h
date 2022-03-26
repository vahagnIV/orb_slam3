//
// Created by vahagn on 26.03.22.
//

#ifndef ORB_SLAM3_TEST_MOCK_MOCK_FRAME_H_
#define ORB_SLAM3_TEST_MOCK_MOCK_FRAME_H_
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <frame/base_frame.h>

namespace orb_slam3 {
namespace test {

class MockFrame : public frame::BaseFrame {
 public:
  MockFrame(TimePoint time_point,
            std::string filename,
            const frame::SensorConstants *sensor_constants,
            size_t id,
            map::Atlas * atlas): frame::BaseFrame(time_point, filename, sensor_constants, id, atlas) { } ;
  MOCK_METHOD(camera::ICamera *, GetCamera, (), (const, override));
  MOCK_METHOD(void, SetCamera, (const camera::ICamera *), (override));
  MOCK_METHOD(frame::FrameType, Type, (), (const, override));
  MOCK_METHOD(void, ListMapPoints, (frame::BaseFrame::MapPointSet &), (const, override));
};

}
}
#endif //ORB_SLAM3_TEST_MOCK_MOCK_FRAME_H_
