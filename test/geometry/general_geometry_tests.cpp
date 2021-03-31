//
// Created by vahagn on 31/03/2021.
//

#include "general_geometry_tests.h"
#include <geometry/utils.h>

namespace orb_slam3 {
namespace test {

TEST_F(GeneralGeometryTests, TriangulateWorksWell) {
  geometry::Pose pose;

  pose.R << 0.967303, 0.0209349, -0.252757,
      -0.0431297, 0.99565, -0.0825917,
      0.249928, 0.0907926, 0.963998;

  pose.T << 1.2, -7.5, 4.1;

  TPoint3D X;
  X << 1.7, -2.5, 4.6;

  HomogenousPoint p1 = X / X[2];
  HomogenousPoint p2 = pose.R * X + pose.T;
  p2 /= p2[2];

  TPoint3D triangulated;
  geometry::utils::Triangulate(pose, p1, p2, triangulated);
  ASSERT_TRUE(std::abs(X[0] - triangulated[0]) < 1e-12);
  ASSERT_TRUE(std::abs(X[1] - triangulated[1]) < 1e-12);
  ASSERT_TRUE(std::abs(X[2] - triangulated[2]) < 1e-12);

  double parallax;
  ASSERT_TRUE(geometry::utils::TriangulateAndValidate(p1, p2, pose, 1e-12, 1e-12, 0.9998, parallax, triangulated));

}

}
}