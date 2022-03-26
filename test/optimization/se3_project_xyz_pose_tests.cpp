//
// Created by vahagn on 26.03.22.
//

#include "se3_project_xyz_pose_tests.h"
#include "g2o/core/jacobian_workspace.h"
#include <optimization/edges/se3_project_xyz_pose.h>
#include <frame/monocular/monocular_frame.h>
#include "mock/mock_frame.h"

namespace orb_slam3 {
namespace test {

TEST(Se3ProjectXyzPoseTests, JacobianIsComputedorrectly) {
  TimePoint time_point;
  std::string filename;
  const frame::SensorConstants *sensor_constants;
  size_t id;
  map::Atlas *atlas;
  MockFrame mock_frame(time_point, filename, sensor_constants, id, atlas);
//  mock_frame.SetStagingPosition(g2o::SE3Quat::)
}

}
}