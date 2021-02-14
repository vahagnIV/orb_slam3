//
// Created by vahagn on 08/02/21.
//

#include <geometry/homography_matrix_estimator.h>
#include <iostream>
#include <math.h>
namespace orb_slam3 {
namespace geometry {
const precision_t HomographyMatrixEstimator::HOMOGRAPHY_THRESHOLD = 5.991;
const precision_t HomographyMatrixEstimator::PARALLAX_THRESHOLD = 0.99998;

bool HomographyMatrixEstimator::FindRTTransformation(const TMatrix33 & homography,
                                                     const std::vector<HomogenousPoint> & kp1,
                                                     const std::vector<HomogenousPoint> & kp2,
                                                     const pairs_t & good_matches,
                                                     std::vector<bool> & out_inliers,
                                                     std::vector<TPoint3D> & out_triangulated,
                                                     TPose & out_pose) const {

  Eigen::JacobiSVD<TMatrix33> svd(homography, Eigen::ComputeFullV | Eigen::ComputeFullU);
  const precision_t d1 = svd.singularValues()[0];
  const precision_t d2 = svd.singularValues()[1];
  const precision_t d3 = svd.singularValues()[2];

  if (d1 / d2 < 1.0001 || d2 / d3 < 1.0001)
    return false;

  const TMatrix33 & U = svd.matrixU();
  const TMatrix33 VT = svd.matrixV().transpose();
  precision_t s = U.determinant() * VT.determinant();

  Solution solution[8];
  FillSolutionsForPositiveD(d1, d2, d3, U, VT, solution, s);
  FillSolutionsForNegativeD(d1, d2, d3, U, VT, solution + 4, s);
  std::vector<TPoint3D> tmp_triangulated;
  for (const Solution & sol: solution) {
    size_t no_good = CheckRT(sol, kp1, kp2, good_matches, out_inliers, tmp_triangulated);

  }

  for (int j = 0; j < 4; ++j) {
    const Solution & sol = solution[j];
    std::cout << " ========= Solution " << j << " ========" << std::endl;
    std::cout << "Determinant R: " << sol.R.determinant() << std::endl;

    TPoint3D probe{100., 100., 100.};
    probe = probe * d2 / probe.dot(sol.n);

    TPoint3D transormed = sol.R * probe + sol.T;
    std::cout << "Transformed " << transormed[0] << " " << transormed[1] << " " << transormed[2] << std::endl;
    std::cout << "Transformed projection " << transormed[0] / transormed[2] << " " << transormed[1] / transormed[2]
              << std::endl;
    std::cout << "Satisfies visibility contraint " << (transormed[2] > 0 ? "true" : "false") << std::endl;
    std::cout << "R " << std::endl << sol.R << std::endl;
    std::cout << "T " << std::endl << sol.T << std::endl;
    std::cout << "n " << std::endl << sol.n << std::endl << std::endl;
  }

  for (int j = 0; j < 4; ++j) {
    const Solution & sol = solution[j + 4];
    std::cout << " ========= Solution " << j << " ========" << std::endl;
    std::cout << "Determinant R: " << sol.R.determinant() << std::endl;

    TPoint3D probe{100., 100., 100.};
    probe = probe * d2 / probe.dot(sol.n);

    TPoint3D transormed = sol.R * probe + sol.T;
    std::cout << "Transformed " << transormed[0] << " " << transormed[1] << " " << transormed[2] << std::endl;
    std::cout << "Transformed projection " << transormed[0] / transormed[2] << " " << transormed[1] / transormed[2]
              << std::endl;
    std::cout << "Satisfies visibility contraint " << (transormed[2] > 0 ? "true" : "false") << std::endl;
    std::cout << "R " << std::endl << sol.R << std::endl;
    std::cout << "T " << std::endl << sol.T << std::endl;
    std::cout << "n " << std::endl << sol.n << std::endl << std::endl;

  }
  return true;

}

void HomographyMatrixEstimator::FillSolutionsForPositiveD(precision_t d1,
                                                          precision_t d2,
                                                          precision_t d3,
                                                          const TMatrix33 & U,
                                                          const TMatrix33 & VT,
                                                          HomographyMatrixEstimator::Solution * solution,
                                                          precision_t s) const {
  precision_t x1 = std::sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
  precision_t x3 = std::sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
  precision_t sin_theta = std::sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / (d1 + d3) / d2;
  precision_t cos_theta = (d2 * d2 + d1 * d3) / (d1 + d3) / d2;
  static precision_t epsilon[4][2] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  for (int i = 0; i < 4; ++i) {
    precision_t epsilon_1 = epsilon[i][0];
    precision_t epsilon_3 = epsilon[i][1];
    precision_t signed_sin = epsilon_1 * epsilon_3 * sin_theta;
    Solution & sol = solution[i];
    sol.R << cos_theta, 0, -signed_sin, 0, 1, 0, signed_sin, 0, cos_theta;
    sol.T << (d1 - d3) * epsilon_1 * x1, 0, (d1 - d3) * epsilon_3 * x3;
    sol.n << epsilon_1 * x1, 0, epsilon_3 * x3;

    sol.R = s * U * sol.R * VT;
    sol.T = U * sol.T;
    sol.n = VT.transpose() * sol.n;
  }
}

void HomographyMatrixEstimator::FillSolutionsForNegativeD(precision_t d1,
                                                          precision_t d2,
                                                          precision_t d3,
                                                          const TMatrix33 & U,
                                                          const TMatrix33 & VT,
                                                          HomographyMatrixEstimator::Solution * solution,
                                                          precision_t s) const {

  precision_t x1 = std::sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
  precision_t x3 = std::sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
  precision_t sin_theta = std::sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / (d1 - d3) / d2;
  precision_t cos_theta = (d1 * d3 - d2 * d2) / (d1 - d3) / d2;
  static precision_t epsilon[4][2] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  for (int i = 0; i < 4; ++i) {
    precision_t epsilon_1 = epsilon[i][0];
    precision_t epsilon_3 = epsilon[i][1];
    precision_t signed_sin = epsilon_1 * epsilon_3 * sin_theta;
    Solution & sol = solution[i];
    sol.R << cos_theta, 0, signed_sin, 0, -1, 0, signed_sin, 0, -cos_theta;
    sol.T << (d1 + d3) * epsilon_1 * x1, 0, (d1 + d3) * epsilon_3 * x3;
    sol.n << epsilon_1 * x1, 0, epsilon_3 * x3;
    sol.R = s * U * sol.R * VT;
    sol.T = U * sol.T;
    sol.n = VT.transpose() * sol.n;
  }
}

void HomographyMatrixEstimator::FindBestHomographyMatrix(const std::vector<HomogenousPoint> & kp1,
                                                         const std::vector<HomogenousPoint> & kp2,
                                                         const pairs_t & good_matches,
                                                         const std::vector<std::vector<size_t>> & good_match_random_idx,
                                                         TMatrix33 & out_homography,
                                                         std::vector<bool> & out_inliers,
                                                         precision_t & out_error) const {
  out_error = std::numeric_limits<precision_t>::max();
  TMatrix33 tmp_homography;

  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const std::vector<size_t> & good_matches_rnd = good_match_random_idx[i];
    std::vector<bool> tmp_inliers(good_matches.size());
    std::fill(tmp_inliers.begin(), tmp_inliers.end(), true);
    FindHomographyMatrix(kp1, kp2, good_matches, good_matches_rnd, tmp_homography);
    precision_t error = ComputeHomographyReprojectionError(tmp_homography, kp1, kp2, good_matches, tmp_inliers, false);
    if (error > 0)
      error += ComputeHomographyReprojectionError(tmp_homography.inverse(), kp1, kp2, good_matches, tmp_inliers, true);
    if (error > 0 && out_error > error) {
      out_homography = tmp_homography;
      out_inliers = tmp_inliers;
      out_error = error;
    }
  }
}

precision_t HomographyMatrixEstimator::ComputeHomographyReprojectionError(const TMatrix33 & h,
                                                                          const std::vector<HomogenousPoint> & kp1,
                                                                          const std::vector<HomogenousPoint> & kp2,
                                                                          const pairs_t & good_matches,
                                                                          std::vector<bool> & out_inliers,
                                                                          bool inverse) const {
  precision_t error = 0;
  for (size_t i = 0; i < good_matches.size(); ++i) {
    if (!out_inliers[i])
      continue;
    const auto & match = good_matches[i];
    const HomogenousPoint & point_from = (inverse ? kp1[match.first] : kp2[match.second]);
    const HomogenousPoint & point_to = (inverse ? kp2[match.second] : kp1[match.first]);

    HomogenousPoint p21 = h * point_from;
    p21 /= p21[2];
    precision_t chi_square2 =
        ((p21[0] - point_to[0]) * (p21[0] - point_to[0]) + (p21[1] - point_to[1]) * (p21[1] - point_to[1]))
            * sigma_squared_inv_;

    if (chi_square2 > HOMOGRAPHY_THRESHOLD) {
      out_inliers[i] = false;
    } else
      error += HOMOGRAPHY_THRESHOLD - chi_square2;
  }
  return error;
}

void HomographyMatrixEstimator::FindHomographyMatrix(const std::vector<HomogenousPoint> & kp1,
                                                     const std::vector<HomogenousPoint> & kp2,
                                                     const std::vector<std::pair<size_t, size_t>> & good_matches,
                                                     const std::vector<size_t> & good_match_random_idx,
                                                     TMatrix33 & out_homography) const {

  Eigen::Matrix<precision_t, Eigen::Dynamic, 9> L;
  L.resize(good_match_random_idx.size() * 2, Eigen::NoChange);

  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const auto & U = kp1[good_matches[good_match_random_idx[i]].first];
    const auto & X = kp2[good_matches[good_match_random_idx[i]].second];

    L(2 * i, 0) = X[0];
    L(2 * i, 1) = X[1];
    L(2 * i, 2) = 1;
    L(2 * i, 3) = 0;
    L(2 * i, 4) = 0;
    L(2 * i, 5) = 0;
    L(2 * i, 6) = -U[0] * X[0];
    L(2 * i, 7) = -U[0] * X[1];
    L(2 * i, 8) = -U[0];

    L(2 * i + 1, 0) = 0;
    L(2 * i + 1, 1) = 0;
    L(2 * i + 1, 2) = 0;
    L(2 * i + 1, 3) = X[0];
    L(2 * i + 1, 4) = X[1];
    L(2 * i + 1, 5) = 1;
    L(2 * i + 1, 6) = -U[1] * X[0];
    L(2 * i + 1, 7) = -U[1] * X[1];
    L(2 * i + 1, 8) = -U[1];
  }

  Eigen::JacobiSVD<Eigen::Matrix<precision_t, Eigen::Dynamic, 9>> svd(L, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (!svd.computeV())
    return;

  const Eigen::Matrix<precision_t, 9, 9> & v = svd.matrixV();

  out_homography << v(0, 8), v(1, 8), v(2, 8),
      v(3, 8), v(4, 8), v(5, 8),
      v(6, 8), v(7, 8), v(8, 8);
  out_homography /= out_homography(2, 2);

}

size_t HomographyMatrixEstimator::CheckRT(const Solution & solution,
                                       const std::vector<HomogenousPoint> & points_to,
                                       const std::vector<HomogenousPoint> & points_from,
                                       const HomographyMatrixEstimator::pairs_t & good_matches,
                                       std::vector<bool> & inliers,
                                       std::vector<TPoint3D> & trinagulated) const {
  size_t count = 0;

  for (size_t i = 0; i < good_matches.size(); ++i) {

    if (!inliers[i])
      continue;
    const auto & match = good_matches[i];
    const HomogenousPoint point_to = points_to[match.first];
    const HomogenousPoint point_from = points_from[match.second];

    TPoint3D triangulated;

    if (!Triangulate(solution, point_to, point_from, triangulated)) {
      inliers[i] = false;
      continue;
    }

    // Eliminate far points
    if (ComputeParallax(triangulated, solution) > PARALLAX_THRESHOLD) {
      inliers[i] = false;
      continue;
    }

    TPoint3D triangulatedC2 = solution.R * triangulated + solution.T;
    precision_t z2_inv = 1. / triangulatedC2[2];
    TPoint2D projectedC2{triangulatedC2[0] * z2_inv, triangulatedC2[1] * z2_inv};

    precision_t error = (point_to[0] - projectedC2[0]) * (point_to[0] - projectedC2[0]) + (point_to[1] - projectedC2[1]) * (point_to[1] - projectedC2[1]);
    if(error > 4 * sigma_threshold_){
      inliers[i] = false;
      continue;
    }
    ++count;

  }
  return count;
}

precision_t HomographyMatrixEstimator::ComputeParallax(const TPoint3D & point,
                                                       const HomographyMatrixEstimator::Solution & solution) const {
  const TVector3D vec1 = point;
  const TVector3D vec2 = point - (solution.T.transpose() * solution.R).transpose();
  return vec1.dot(vec2) / vec1.norm() / vec2.norm();
}

/*
 * Zisserman 12.2 Linear triangulation methods p. 312
 * The preojection of a 3D point X = (X,Y,Z,1) is given by a 3x4 projection matrix P, i.e. pt =~ PX,
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
bool HomographyMatrixEstimator::Triangulate(const HomographyMatrixEstimator::Solution & sol,
                                            const HomogenousPoint & pt1,
                                            const HomogenousPoint & pt2,
                                            TPoint3D & out_trinagulated) const {
  Eigen::Matrix<precision_t, 4, 4, Eigen::RowMajor> A;
  A << 0, -1, pt1[1], 0,
      -1, 0, pt1[0], 0,
      pt2[1] * sol.R(2, 0) - sol.R(1, 0), pt2[1] * sol.R(2, 1) - sol.R(1, 1), pt2[1] * sol.R(2, 2) - sol.R(1, 2),
      pt2[1] * sol.T[2] - sol.T[1],
      pt2[0] * sol.R(2, 0) - sol.R(0, 0), pt2[0] * sol.R(2, 1) - sol.R(0, 1), pt2[0] * sol.R(2, 2) - sol.R(0, 2),
      pt2[0] * sol.T[2] - sol.T[0];

  Eigen::JacobiSVD<decltype(A)> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  precision_t l_inv = 1 / svd.matrixV()(3, 3);
  out_trinagulated << svd.matrixV()(0, 3) * l_inv, svd.matrixV()(1, 3) * l_inv, svd.matrixV()(2, 3) * l_inv;

  return std::isfinite(out_trinagulated[0]) && std::isfinite(out_trinagulated[1]) && std::isfinite(out_trinagulated[2]);
}

}
}