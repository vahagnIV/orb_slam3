//
// Created by vahagn on 30/04/2021.
//

#include "utils_tests.h"
#include "test_utils.h"
#include <geometry/utils.h>

namespace orb_slam3 {
namespace test {

TEST(UtilsTests, TriangulateCorrectlyFindPoint) {
  geometry::Pose pose;
  pose.R = GetRotationMatrixRollPitchYaw(M_PI / 180 * 3, M_PI / 180 * 4, M_PI / 180 * 5);
  pose.T = TVector3D{0.7, 1.2, 0.7};

  TPoint3D point{4, -3, 18};

  HomogenousPoint projection_from{point.x() / point.z(), point.y() / point.z(), 1};
  TPoint3D transformed = pose.Transform(point);
  HomogenousPoint projection_to{transformed.x() / transformed.z(), transformed.y() / transformed.z(), 1};

  TPoint3D triangulated;
  ASSERT_TRUE(geometry::utils::Triangulate(pose, projection_from, projection_to, triangulated));
  ASSERT_TRUE((triangulated - point).norm() < 1e-8);
}

TEST(UtilsTests, GetEssentialMatrixFromPoseReturnsCorrectEssentialMatrix) {
  geometry::Pose pose;
  pose.R = GetRotationMatrixRollPitchYaw(M_PI / 180 * 3, M_PI / 180 * 4, M_PI / 180 * 5);
  pose.T = TVector3D{0.7, 1.2, 0.7};

  auto E = GetEssentialMatrixFromPose(pose);

  TPoint3D point_from1{1, 7, 6};
  TPoint3D point_from2{2, 5, 9};
  TPoint3D point_from3{3, 4, 2};
  TPoint3D point_from4{0, 0, 0};

  TPoint3D point_to1 = pose.Transform(point_from1);
  TPoint3D point_to2 = pose.Transform(point_from2);
  TPoint3D point_to3 = pose.Transform(point_from3);
  TPoint3D point_to4 = pose.Transform(point_from4);

  ASSERT_TRUE(point_to1.dot(E * point_from1) < 1e-14);
  ASSERT_TRUE(point_to2.dot(E * point_from2) < 1e-14);
  ASSERT_TRUE(point_to3.dot(E * point_from3) < 1e-14);
  ASSERT_TRUE(point_to4.dot(E * point_from4) < 1e-14);

}

}
}