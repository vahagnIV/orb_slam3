//
// Created by vahagn on 26.03.22.
//

#include "se3_project_xyz_pose_tests.h"
#include "g2o/core/jacobian_workspace.h"
#include <optimization/edges/se3_project_xyz_pose.h>
#include <optimization/vertices/map_point_vertex.h>
#include <optimization/vertices/frame_vertex.h>
#include <frame/monocular/monocular_frame.h>
#include <map/map_point.h>
#include "camera/monocular_camera.h"
#include "camera/distortions/fish_eye.h"
#include "mock/mock_frame.h"
#include "test_utils.h"

namespace orb_slam3 {
namespace test {

using ::testing::Return;

TEST(Se3ProjectXyzPoseTests, JacobianIsComputedorrectly) {

  //  ====== frame ======
  TimePoint time_point;
  std::string filename;
  const frame::SensorConstants *sensor_constants;
  size_t id;
  map::Atlas *atlas;
  MockFrame mock_frame(time_point, filename, sensor_constants, id, atlas);
  mock_frame.SetIdentity();
//  mock_frame.SetStagingPosition(GetRotationMatrixRollPitchYaw(1, 0.5, 2.3333), TVector3D{1, 2, 4});

  // ===== camera ====
  camera::MonocularCamera camera(1000, 1000);
  camera.SetFx(808);
  camera.SetFy(812);
  camera.SetCx(498);
  camera.SetCy(501);

  auto distortion = new camera::FishEye();
  distortion->SetK1(8.0260387888370061e-02);
  distortion->SetK2(-2.3658494730230581e-01);
  distortion->SetK3(6.0946691237477612e-04);
  distortion->SetK4(3.1997222204038837e-04);
  camera.SetDistortionModel(distortion);
  ON_CALL(mock_frame, GetCamera).WillByDefault(Return(&camera));

  TPoint3D point;
  map::MapPoint map_point(point, 1, 1, 1, nullptr);

  optimization::vertices::FrameVertex frame_vertex(&mock_frame);
  optimization::vertices::MapPointVertex map_point_vertex(&map_point);

  optimization::edges::SE3ProjectXYZPose e(&camera, 1e-4);
  e.setVertex(0, &frame_vertex);
  e.setVertex(1, &map_point_vertex);

  g2o::JacobianWorkspace jacobianWorkspace;
  g2o::JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(&e);
  numericJacobianWorkspace.allocate();
  for (int i = 0; i < 10; ++i) {
    point.setRandom();

    map_point.SetStagingPosition(point);
    TPoint3D local = mock_frame.GetStagingPosition().Transform(map_point.GetStagingPosition());
    TPoint2D measurement;
    camera.ProjectAndDistort(local, measurement);
    TPoint2D measurement_distortion;
    measurement_distortion.setRandom();
    measurement_distortion /= measurement_distortion.maxCoeff();

  }

}

}
}