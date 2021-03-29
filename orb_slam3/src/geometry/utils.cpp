//
// Created by vahagn on 22.03.21.
//

#include "geometry/utils.h"

namespace orb_slam3 {
namespace geometry {
namespace utils {

TMatrix33 SkewSymmetricMatrix(const TVector3D & vector) {
  TMatrix33 result;
  result << 0, -vector[2], vector[1],
      vector[2], 0, -vector[0],
      -vector[1], vector[0], 0;
  return result;
}

void ComputeRelativeTransformation(const Pose & pose_to,
                                   const Pose & pose_from,
                                   Pose & out_pose) {
  out_pose.R = pose_to.R * pose_from.R.transpose();
  out_pose.T = -out_pose.R * pose_from.T + pose_to.T;
}

/*
 * Zisserman 12.2 Linear triangulation methods p. 312
 * The projection of a 3D point X = (X,Y,Z,1) is given by a 3x4 projection matrix P, i.e. pt =~ PX,
 * where =~ means the projective equality. To eliminatie the multiplier we can rewrite the equation as
 * a vector product pt x PX = 0. This is an overdetermined set of 3 equations.
 *
 * For pt1 the matrix P reads
 * 1 0 0 0
 * 0 1 0 0
 * 0 0 1 0
 *
 * while for pt2
 *
 * r00 r01 r01 t0
 * r10 r11 r12 t1
 * r20 r21 r22 t2
 *
 * Combining the first 2 equations for each point we get
 * AX = 0,
 *
 * where A is a 4x4 matrix such that
 * A[0,.] = pt1.y * P1.row(2) - P1.row(1)
 * A[1,.] = pt1.x * P1.row(2) - P1.row(0)
 * A[2,.] = pt2.y * P2.row(2) - P2.row(1)
 * A[3,.] = pt2.x * P2.row(2) - P2.row(0)
 *
 * The solution of this equation is the column of right singular matrix in SVD that corresponds to the minimal
 * singular value. In Eigen the singular values are sorted sorted. Therefore it is the last column.
 * */
bool Triangulate(const Pose & pose,
                 const HomogenousPoint & point_from,
                 const HomogenousPoint & point_to,
                 TPoint3D & out_trinagulated) {
  Eigen::Matrix<precision_t, 4, 4, Eigen::RowMajor> A;

  A << 0, -1, point_from[1], 0,
      -1, 0, point_from[0], 0,
      point_to[1] * pose.R(2, 0) - pose.R(1, 0), point_to[1] * pose.R(2, 1) - pose.R(1, 1), point_to[1] * pose.R(2, 2)
      - pose.R(1, 2), point_to[1] * pose.T[2] - pose.T[1],
      point_to[0] * pose.R(2, 0) - pose.R(0, 0), point_to[0] * pose.R(2, 1) - pose.R(0, 1), point_to[0] * pose.R(2, 2)
      - pose.R(0, 2), point_to[0] * pose.T[2] - pose.T[0];

  Eigen::JacobiSVD<decltype(A)> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  precision_t l_inv = 1 / svd.matrixV()(3, 3);
  out_trinagulated << svd.matrixV()(0, 3) * l_inv, svd.matrixV()(1, 3) * l_inv, svd.matrixV()(2, 3) * l_inv;

  return std::isfinite(out_trinagulated[0]) && std::isfinite(out_trinagulated[1]) && std::isfinite(out_trinagulated[2]);
}

precision_t ComputeParallax(const Pose & pose, const TPoint3D & point) {
  const TVector3D vec1 = point;
  const TVector3D vec2 = point - (pose.T.transpose() * pose.R).transpose();
  return vec1.dot(vec2) / vec1.norm() / vec2.norm();
}

precision_t ComputeReprojectionError(const TPoint3D & point, const HomogenousPoint & original_point) {
  precision_t z2_inv = 1. / point[2];
  TPoint2D projected{point[0] * z2_inv, point[1] * z2_inv};

  return (original_point[0] - projected[0]) * (original_point[0] - projected[0])
      + (original_point[1] - projected[1]) * (original_point[1] - projected[1]);
}

bool TriangulateAndValidate(const HomogenousPoint & point_from,
                            const HomogenousPoint & point_to,
                            const Pose & pose,
                            precision_t reprojection_threshold_to,
                            precision_t reprojection_threshold_from,
                            precision_t parallax_threshold,
                            precision_t & out_parallax,
                            TPoint3D & out_triangulated) {
  if (!utils::Triangulate(pose, point_from, point_to, out_triangulated))
    return false;

  if (out_triangulated[2] < 0)
    return false;

  out_parallax = utils::ComputeParallax(pose, out_triangulated);
  if (out_parallax > parallax_threshold || std::isnan(out_parallax))
    return false;

  const TVector3D triangulated2 = pose.Transform(out_triangulated);
  if (triangulated2[2] < 0) return false;

  if (utils::ComputeReprojectionError(out_triangulated, point_from) > reprojection_threshold_from
      || utils::ComputeReprojectionError(triangulated2, point_to) > reprojection_threshold_to)
    return false;

  return true;
}

}
}
}