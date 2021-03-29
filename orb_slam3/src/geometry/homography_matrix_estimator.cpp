//
// Created by vahagn on 08/02/21.
//

// == stl ===
#include <cmath>

// == orb-slam3 ===
#include <geometry/homography_matrix_estimator.h>
#include <geometry/utils.h>

namespace orb_slam3 {
namespace geometry {

const precision_t HomographyMatrixEstimator::HOMOGRAPHY_SCORE = 5.991;
const precision_t HomographyMatrixEstimator::PARALLAX_THRESHOLD = 0.99998;

bool HomographyMatrixEstimator::FindRTTransformation(const TMatrix33 &homography,
                                                     const std::vector<HomogenousPoint> &points_to,
                                                     const std::vector<HomogenousPoint> &points_from,
                                                     const std::vector<features::Match> &matches,
                                                     std::vector<bool> &out_inliers,
                                                     std::vector<TPoint3D> &out_triangulated,
                                                     Pose & out_pose) const {

  Eigen::JacobiSVD<TMatrix33> svd(homography, Eigen::ComputeFullV | Eigen::ComputeFullU);
  const precision_t d1 = svd.singularValues()[0];
  const precision_t d2 = svd.singularValues()[1];
  const precision_t d3 = svd.singularValues()[2];

  if (d1 / d2 < 1.0001 || d2 / d3 < 1.0001)
    return false;

  const TMatrix33 &U = svd.matrixU();
  const TMatrix33 VT = svd.matrixV().transpose();
  precision_t s = U.determinant() * VT.determinant();

  Pose solution[8];
  FillSolutionsForPositiveD(d1, d2, d3, U, VT, solution, s);
  FillSolutionsForNegativeD(d1, d2, d3, U, VT, solution + 4, s);

  size_t best_count = 0, second_best_count = 0;
  precision_t best_parallax = -1;

//  std::vector<bool> original_inliers = out_inliers;
  for (const auto &sol : solution) {
    std::vector<TPoint3D> tmp_triangulated;
    std::vector<bool> tmp_inliers(matches.size(), true);
//    std::vector<bool> tmp_inliers = original_inliers;
    precision_t parallax;
    size_t no_good = CheckRT(sol, points_to, points_from, matches, tmp_inliers, parallax, tmp_triangulated);
    if (best_count < no_good) {
      second_best_count = best_count;
      best_count = no_good;
      out_triangulated = tmp_triangulated;
      out_inliers = tmp_inliers;
      out_pose = sol;
      best_parallax = parallax;
    } else
      second_best_count = std::max(second_best_count, no_good);
  }

  //secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N
  // TODO: address this issue
  return second_best_count < 0.75 * best_count && best_parallax < 0.995 && best_count > 30;
}

void HomographyMatrixEstimator::FillSolutionsForPositiveD(precision_t d1,
                                                          precision_t d2,
                                                          precision_t d3,
                                                          const TMatrix33 &U,
                                                          const TMatrix33 &VT,
                                                          Pose *solution,
                                                          precision_t s) noexcept {
  const precision_t x1 = std::sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
  const precision_t x3 = std::sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
  const precision_t sin_theta = std::sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 + d3) * d2);
  const precision_t cos_theta = (d2 * d2 + d1 * d3) / ((d1 + d3) * d2);
  static precision_t epsilon[4][2] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  for (int i = 0; i < 4; ++i) {
    const precision_t epsilon_1 = epsilon[i][0];
    const precision_t epsilon_3 = epsilon[i][1];
    const precision_t signed_sin = epsilon_1 * epsilon_3 * sin_theta;
    Pose &sol = solution[i];
    sol.R << cos_theta, 0, -signed_sin, 0, 1, 0, signed_sin, 0, cos_theta;
    sol.T << (d1 - d3) * epsilon_1 * x1, 0, -(d1 - d3) * epsilon_3 * x3;

    sol.R = s * U * sol.R * VT;
    sol.T = U * sol.T;
    sol.T /= sol.T.norm();
  }
}

void HomographyMatrixEstimator::FillSolutionsForNegativeD(precision_t d1,
                                                          precision_t d2,
                                                          precision_t d3,
                                                          const TMatrix33 &U,
                                                          const TMatrix33 &VT,
                                                          Pose *solution,
                                                          precision_t s) noexcept {

  const precision_t x1 = std::sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
  const precision_t x3 = std::sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
  const precision_t sin_theta = std::sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);
  const precision_t cos_theta = (d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
  static precision_t epsilon[4][2] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  for (int i = 0; i < 4; ++i) {
    const precision_t epsilon_1 = epsilon[i][0];
    const precision_t epsilon_3 = epsilon[i][1];
    const precision_t signed_sin = epsilon_1 * epsilon_3 * sin_theta;
    Pose &sol = solution[i];
    sol.R << cos_theta, 0, signed_sin, 0, -1, 0, signed_sin, 0, -cos_theta;
    sol.T << (d1 + d3) * epsilon_1 * x1, 0, (d1 + d3) * epsilon_3 * x3;
    sol.R = s * U * sol.R * VT;
    sol.T = U * sol.T;
    sol.T /= sol.T.norm();
  }
}

void HomographyMatrixEstimator::FindBestHomographyMatrix(const std::vector<HomogenousPoint> &points_to,
                                                         const std::vector<HomogenousPoint> &points_from,
                                                         const std::vector<features::Match> &matches,
                                                         const std::vector<std::vector<size_t>> &good_match_random_idx,
                                                         TMatrix33 &out_homography,
                                                         std::vector<bool> &out_inliers,
                                                         precision_t &out_score) const {
  out_score = std::numeric_limits<precision_t>::min();
  TMatrix33 tmp_homography;

  for (const auto &good_matches_rnd : good_match_random_idx) {
    std::vector<bool> tmp_inliers(matches.size());
    std::fill(tmp_inliers.begin(), tmp_inliers.end(), true);
    FindHomographyMatrix(points_to, points_from, matches, good_matches_rnd, tmp_homography);
    precision_t score =
        ComputeHomographyReprojectionError(tmp_homography, points_to, points_from, matches, tmp_inliers, false);
    if (score > 0)
      score += ComputeHomographyReprojectionError(tmp_homography.inverse(),
                                                  points_to,
                                                  points_from,
                                                  matches,
                                                  tmp_inliers,
                                                  true);
    if (score > 0 && score > out_score) {
      out_homography = tmp_homography;
      out_inliers = tmp_inliers;
      out_score = score;
    }
  }
}

precision_t HomographyMatrixEstimator::ComputeHomographyReprojectionError(const TMatrix33 &h,
                                                                          const std::vector<HomogenousPoint> &points_to,
                                                                          const std::vector<HomogenousPoint> &points_from,
                                                                          const std::vector<features::Match> &matches,
                                                                          std::vector<bool> &out_inliers,
                                                                          bool inverse) const {
  precision_t score = 0;
  for (size_t i = 0; i < matches.size(); ++i) {
    if (!out_inliers[i])
      continue;
    const auto &match = matches[i];
    const HomogenousPoint &point_from = (inverse ? points_to[match.to_idx] : points_from[match.from_idx]);
    const HomogenousPoint &point_to = (inverse ? points_from[match.from_idx] : points_to[match.to_idx]);

    HomogenousPoint p21 = h * point_from;
    p21 /= p21[2];
    precision_t chi_square2 =
        ((p21[0] - point_to[0]) * (p21[0] - point_to[0]) + (p21[1] - point_to[1]) * (p21[1] - point_to[1]))
            * sigma_squared_inv_;

    if (chi_square2 > HOMOGRAPHY_SCORE) {
      out_inliers[i] = false;
    } else
      score += HOMOGRAPHY_SCORE - chi_square2;
  }
  return score;
}

void HomographyMatrixEstimator::FindHomographyMatrix(const std::vector<HomogenousPoint> &points_to,
                                                     const std::vector<HomogenousPoint> &points_from,
                                                     const std::vector<features::Match> &matches,
                                                     const std::vector<size_t> &good_match_random_idx,
                                                     TMatrix33 &out_homography) {

  Eigen::Matrix<precision_t, Eigen::Dynamic, 9> L;
  L.resize(good_match_random_idx.size() * 2, Eigen::NoChange);

  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const auto &U = points_to[matches[good_match_random_idx[i]].to_idx];
    const auto &X = points_from[matches[good_match_random_idx[i]].from_idx];

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

  const Eigen::Matrix<precision_t, 9, 9> &v = svd.matrixV();

  out_homography << v(0, 8), v(1, 8), v(2, 8),
      v(3, 8), v(4, 8), v(5, 8),
      v(6, 8), v(7, 8), v(8, 8);
  out_homography /= out_homography(2, 2);

}

size_t HomographyMatrixEstimator::CheckRT(const Pose &solution,
                                          const std::vector<HomogenousPoint> &points_to,
                                          const std::vector<HomogenousPoint> &points_from,
                                          const std::vector<features::Match> &matches,
                                          std::vector<bool> &inliers,
                                          precision_t &out_parallax,
                                          std::vector<TPoint3D> &out_triangulated) const {
  size_t count = 0;
  out_triangulated.resize(matches.size());
  std::vector<precision_t> triangulated_parallax;
  triangulated_parallax.reserve(matches.size());

  for (size_t i = 0; i < matches.size(); ++i) {

    if (!inliers[i])
      continue;

    const auto &match = matches[i];
    const HomogenousPoint point_to = points_to[match.to_idx];
    const HomogenousPoint point_from = points_from[match.from_idx];

    TPoint3D &triangulated = out_triangulated[i];

    precision_t point_parallax;
    if (!utils::TriangulateAndValidate(point_from,
                                       point_to,
                                       solution,
                                       4 * sigma_threshold__square_,
                                       4 * sigma_threshold__square_,
                                       PARALLAX_THRESHOLD,
                                       point_parallax,
                                       triangulated))
      continue;

    triangulated_parallax.push_back(point_parallax);
    ++count;
  }
  if (!count)
    return count;

  std::sort(triangulated_parallax.begin(), triangulated_parallax.end());
  out_parallax =
      std::acos(triangulated_parallax.size() < 50 ? triangulated_parallax.back() : triangulated_parallax[50]);

  return count;
}

}
}