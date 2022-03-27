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
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include <map/map.h>

namespace g2o {
template<typename EdgeType>
void evaluateJacobian(EdgeType &e, JacobianWorkspace &jacobianWorkspace,
                      g2o::JacobianWorkspace &numericJacobianWorkspace) {
  // calling the analytic Jacobian but writing to the numeric workspace
  e.BaseBinaryEdge<EdgeType::Dimension, typename EdgeType::Measurement,
                   typename EdgeType::VertexXiType,
                   typename EdgeType::VertexXjType>::linearizeOplus(numericJacobianWorkspace);
  // copy result into analytic workspace
  jacobianWorkspace = numericJacobianWorkspace;

  // compute the numeric Jacobian into the numericJacobianWorkspace workspace as
  // setup by the previous call
  e.BaseBinaryEdge<EdgeType::Dimension, typename EdgeType::Measurement,
                   typename EdgeType::VertexXiType,
                   typename EdgeType::VertexXjType>::linearizeOplus();

  // compare the two Jacobians
  for (int i = 0; i < 2; ++i) {
    number_t *n = numericJacobianWorkspace.workspaceForVertex(i);
    number_t *a = jacobianWorkspace.workspaceForVertex(i);
    int numElems = EdgeType::Dimension;
    if (i == 0)
      numElems *= EdgeType::VertexXiType::Dimension;
    else
      numElems *= EdgeType::VertexXjType::Dimension;
    for (int j = 0; j < numElems; ++j) {
      EXPECT_NEAR(n[j], a[j], 1e-3);
    }
  }
}
}

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
  camera.SetFy(808);
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
  map::Map mp(nullptr);
  map::MapPoint map_point(point, 1, 1, 1, &mp);

  optimization::vertices::FrameVertex frame_vertex(&mock_frame);
  optimization::vertices::MapPointVertex map_point_vertex(&map_point);

  optimization::edges::SE3ProjectXYZPose e(&camera, 1e-4);
  e.setVertex(0, &frame_vertex);
  e.setVertex(1, &map_point_vertex);
  e.setInformation(TMatrix22::Identity());

  g2o::JacobianWorkspace jacobianWorkspace;
  g2o::JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(&e);
  numericJacobianWorkspace.allocate();
  for (int i = 0; i < 10; ++i) {
    mock_frame.SetStagingPosition(
        GetRotationMatrixRollPitchYaw(DoubleRand(-M_PI / 2, M_PI / 2),
                                      DoubleRand(-M_PI / 2, M_PI / 2),
                                      DoubleRand(-M_PI / 2, M_PI / 2)),
        GenerateRandom3DPoint(0, 0, 0, 10, 10, 10));
    frame_vertex.setEstimate(g2o::SE3Quat(mock_frame.GetStagingPosition().R, mock_frame.GetStagingPosition().T));

    TPoint3D local = GenerateRandom3DPoint(-20, -20, 0.5, 20, 20, 60);
    TPoint2D measurement;
    camera.ProjectAndDistort(local, measurement);
    TPoint2D measurement_distortion = GenerateRandom2DPoint(-0.2, -0.2, 0.2, 0.2);

//    measurement += measurement_distortion;
    map_point.SetStagingPosition(mock_frame.GetStagingPosition().GetInversePose().Transform(local));
    map_point_vertex.setEstimate(map_point.GetStagingPosition());
    evaluateJacobian(e, jacobianWorkspace, numericJacobianWorkspace);
  }

}

}
}