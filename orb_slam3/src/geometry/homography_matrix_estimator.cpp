//
// Created by vahagn on 08/02/21.
//

#include <geometry/homography_matrix_estimator.h>
#include <iostream>
namespace orb_slam3 {
namespace geometry {
const precision_t HomographyMatrixEstimator::HOMOGRAPHY_THRESHOLD = 5.991;

bool HomographyMatrixEstimator::FindRTTransformation(const TMatrix33 & homography,
                                                     const std::vector<TPoint3D> & kp1,
                                                     const std::vector<TPoint3D> & kp2,
                                                     const pairs_t & good_matches,
                                                     const std::vector<bool> & out_inliers,
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
    int no_good = CheckRT(sol, kp1, kp2, good_matches, out_inliers, tmp_triangulated);

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

void HomographyMatrixEstimator::FindBestHomographyMatrix(const std::vector<TPoint3D> & kp1,
                                                         const std::vector<TPoint3D> & kp2,
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
                                                                          const std::vector<TPoint3D> & kp1,
                                                                          const std::vector<TPoint3D> & kp2,
                                                                          const pairs_t & good_matches,
                                                                          std::vector<bool> & out_inliers,
                                                                          bool inverse) const {
  precision_t error = 0;
  for (size_t i = 0; i < good_matches.size(); ++i) {
    if (!out_inliers[i])
      continue;
    const auto & match = good_matches[i];
    const TPoint3D & point_from = (inverse ? kp1[match.first] : kp2[match.second]);
    const TPoint3D & point_to = (inverse ? kp2[match.second] : kp1[match.first]);

    TPoint3D p21 = h * point_from;
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

void HomographyMatrixEstimator::FindHomographyMatrix(const std::vector<TPoint3D> & kp1,
                                                     const std::vector<TPoint3D> & kp2,
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

int HomographyMatrixEstimator::CheckRT(const Solution & solution,
                                       const std::vector<TPoint3D> & kp1,
                                       const std::vector<TPoint3D> & kp2,
                                       const HomographyMatrixEstimator::pairs_t & good_matches,
                                       const std::vector<bool> & inliers,
                                       std::vector<TPoint3D> & trinagulated) const {

  for (size_t i = 0; i < good_matches.size(); ++i) {
    if (!inliers[i])
      continue;
    const auto & match = good_matches[i];
    TPoint3D transformed = solution.R * kp2[match.second] + solution.T;


  }
  return 0;
}

}
}