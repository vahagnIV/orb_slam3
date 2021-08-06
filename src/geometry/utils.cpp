//
// Created by vahagn on 22.03.21.
//

#include "utils.h"
#include <unordered_set>

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
 * singular value. In Eigen the singular values are stored sorted. Therefore it is the last column.
 * */
bool Triangulate(const Pose & pose,
                 const HomogenousPoint & point_from,
                 const HomogenousPoint & point_to,
                 TPoint3D & out_trinagulated) {
  Eigen::Matrix<precision_t, 4, 4, Eigen::RowMajor> A;
  assert(point_from.z() == 1);
  assert(point_to.z() == 1);
  A << -1, 0, point_from.x(), 0,
      0, -1, point_from.y(), 0,
      point_to.x() * pose.R(2, 0) - pose.R(0, 0), point_to.x() * pose.R(2, 1) - pose.R(0, 1),
      point_to.x() * pose.R(2, 2)
          - pose.R(0, 2), point_to.x() * pose.T.z() - pose.T.x(),
      point_to.y() * pose.R(2, 0) - pose.R(1, 0), point_to.y() * pose.R(2, 1) - pose.R(1, 1),
      point_to.y() * pose.R(2, 2)
          - pose.R(1, 2), point_to.y() * pose.T.z() - pose.T.y();

  Eigen::JacobiSVD<decltype(A)> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  precision_t l_inv = 1 / svd.matrixV()(3, 3);
  out_trinagulated << svd.matrixV()(0, 3) * l_inv, svd.matrixV()(1, 3) * l_inv, svd.matrixV()(2, 3) * l_inv;

  return std::isfinite(out_trinagulated[0]) && std::isfinite(out_trinagulated[1]) && std::isfinite(out_trinagulated[2]);
}

precision_t ComputeCosParallax(const Pose & pose, const TPoint3D & point) {
  const TVector3D & vec1 = point;
  const TVector3D vec2 = point - (pose.T.transpose() * pose.R).transpose();
  return vec1.dot(vec2) / vec1.norm() / vec2.norm();
}

precision_t ComputeReprojectionError(const HomogenousPoint & point, const HomogenousPoint & original_point) {
  precision_t zpoint_inv = 1. / point[2];
  precision_t zoriginal_inv = 1. / original_point[2];
  precision_t error_x = point[0] * zpoint_inv - original_point[0] * zoriginal_inv;
  precision_t error_y = point[1] * zpoint_inv - original_point[1] * zoriginal_inv;

  return error_x * error_x + error_y * error_y;
}

precision_t ComputeReprojectionError(const TPoint3D & point,
                                     const TPoint2D & projection,
                                     const camera::MonocularCamera * camera) {
  TPoint2D predicted_projection;
  camera->ProjectAndDistort(point, predicted_projection);
  return (predicted_projection - projection).squaredNorm();
}

bool TriangulateAndValidate(const HomogenousPoint & point_from,
                            const HomogenousPoint & point_to,
                            const Pose & pose,
                            precision_t reprojection_threshold_to,
                            precision_t reprojection_threshold_from,
                            precision_t parallax_cos_threshold,
                            precision_t & out_cos_parallax,
                            TPoint3D & out_triangulated) {
  if (!utils::Triangulate(pose, point_from, point_to, out_triangulated))
    return false;

  if (out_triangulated[2] < 0)
    return false;

  out_cos_parallax = utils::ComputeCosParallax(pose, out_triangulated);
  if (out_cos_parallax > parallax_cos_threshold || std::isnan(out_cos_parallax))
    return false;

  const TVector3D triangulated2 = pose.Transform(out_triangulated);
  if (triangulated2[2] < 0) return false;

  if (utils::ComputeReprojectionError(out_triangulated, point_from) > reprojection_threshold_from
      || utils::ComputeReprojectionError(triangulated2, point_to) > reprojection_threshold_to)
    return false;

  return true;
}

bool ValidateTriangulatedPoint(const TPoint3D & triangulated,
                               const camera::MonocularCamera * camera_from,
                               const camera::MonocularCamera * camera_to,
                               const TPoint2D & point_from_projection,
                               const TPoint2D & point_to_projection,
                               const Pose & pose,
                               precision_t reprojection_error) {

  if (triangulated.z() < 0)
    return false;

  const TVector3D triangulated2 = pose.Transform(triangulated);
  if (triangulated2.z() < 0) return false;

  if (utils::ComputeReprojectionError(triangulated, point_from_projection, camera_from) > reprojection_error
      || utils::ComputeReprojectionError(triangulated2, point_to_projection, camera_to) > reprojection_error)
    return false;

  return true;
}

}
}
}