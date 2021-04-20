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


bool HomographyMatrixEstimator::FindPose(const TMatrix33 & homography,
                                         const std::vector<HomogenousPoint> & points_to,
                                         const std::vector<HomogenousPoint> & points_from,
                                         const std::unordered_map<std::size_t, std::size_t> & matches,
                                         std::unordered_set<size_t> & out_inliers,
                                         std::unordered_map<std::size_t, TPoint3D> & out_triangulated,
                                         Pose & out_pose) const {

  Eigen::JacobiSVD<TMatrix33> svd(homography, Eigen::ComputeFullV | Eigen::ComputeFullU);
  const precision_t d1 = svd.singularValues()[0];
  const precision_t d2 = svd.singularValues()[1];
  const precision_t d3 = svd.singularValues()[2];

  if (d1 / d2 < 1.0001 || d2 / d3 < 1.0001)
    return false;

  const TMatrix33 & U = svd.matrixU();
  const TMatrix33 VT = svd.matrixV().transpose();
  precision_t s = U.determinant() * VT.determinant();

  std::vector<Pose> candidate_solutions(8);
  FillSolutionsForPositiveD(d1, d2, d3, U, VT, candidate_solutions.begin(), candidate_solutions.begin() + 4, s);
  FillSolutionsForNegativeD(d1, d2, d3, U, VT, candidate_solutions.begin() + 4, candidate_solutions.end(), s);

  return this->FindCorrectPose(candidate_solutions, points_to, points_from, matches, out_inliers,
                               out_triangulated, out_pose);
}

void HomographyMatrixEstimator::FillSolutionsForPositiveD(precision_t d1,
                                                          precision_t d2,
                                                          precision_t d3,
                                                          const TMatrix33 & U,
                                                          const TMatrix33 & VT,
                                                          std::vector<Pose>::iterator start,
                                                          std::vector<Pose>::iterator end,
                                                          precision_t s) noexcept {
  const precision_t x1 = std::sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
  const precision_t x3 = std::sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
  const precision_t sin_theta = std::sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 + d3) * d2);
  const precision_t cos_theta = (d2 * d2 + d1 * d3) / ((d1 + d3) * d2);
  static precision_t epsilon[4][2] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  int i = 0;
  for (auto sol = start; sol != end; ++sol) {
    const precision_t epsilon_1 = epsilon[i][0];
    const precision_t epsilon_3 = epsilon[i][1];
    const precision_t signed_sin = epsilon_1 * epsilon_3 * sin_theta;

    sol->R << cos_theta, 0, -signed_sin, 0, 1, 0, signed_sin, 0, cos_theta;
    sol->T << (d1 - d3) * epsilon_1 * x1, 0, -(d1 - d3) * epsilon_3 * x3;

    sol->R = s * U * sol->R * VT;
    sol->T = U * sol->T;
    sol->T /= sol->T.norm();
    ++i;
  }
}

void HomographyMatrixEstimator::FillSolutionsForNegativeD(precision_t d1,
                                                          precision_t d2,
                                                          precision_t d3,
                                                          const TMatrix33 & U,
                                                          const TMatrix33 & VT,
                                                          std::vector<Pose>::iterator start,
                                                          std::vector<Pose>::iterator end,
                                                          precision_t s) noexcept {

  const precision_t x1 = std::sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
  const precision_t x3 = std::sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
  const precision_t sin_theta = std::sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);
  const precision_t cos_theta = (d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
  static precision_t epsilon[4][2] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  int i = 0;
  for (auto sol = start; sol != end; ++sol) {
    const precision_t epsilon_1 = epsilon[i][0];
    const precision_t epsilon_3 = epsilon[i][1];
    const precision_t signed_sin = epsilon_1 * epsilon_3 * sin_theta;
    sol->R << cos_theta, 0, signed_sin, 0, -1, 0, signed_sin, 0, -cos_theta;
    sol->T << (d1 + d3) * epsilon_1 * x1, 0, (d1 + d3) * epsilon_3 * x3;
    sol->R = s * U * sol->R * VT;
    sol->T = U * sol->T;
    sol->T /= sol->T.norm();
    ++i;
  }
}

void HomographyMatrixEstimator::FindBestHomographyMatrix(const std::vector<HomogenousPoint> & points_to,
                                                         const std::vector<HomogenousPoint> & points_from,
                                                         const std::unordered_map<std::size_t, std::size_t> & matches,
                                                         const std::vector<std::vector<size_t>> & good_match_random_idx,
                                                         TMatrix33 & out_homography,
                                                         std::unordered_set<std::size_t> & out_inliers,
                                                         precision_t & out_score) const {
  out_score = std::numeric_limits<precision_t>::min();
  TMatrix33 tmp_homography;

  for (const auto & good_matches_rnd : good_match_random_idx) {
    std::unordered_set<std::size_t> tmp_inliers;
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

precision_t HomographyMatrixEstimator::ComputeHomographyReprojectionError(const TMatrix33 & h,
                                                                          const std::vector<HomogenousPoint> & points_to,
                                                                          const std::vector<HomogenousPoint> & points_from,
                                                                          const std::unordered_map<std::size_t, std::size_t> & matches,
                                                                          std::unordered_set<std::size_t> & out_inliers,
                                                                          bool inverse) const {
  precision_t score = 0;
  typedef std::unordered_map<std::size_t, std::size_t>::const_iterator I;
  for (I i = matches.begin(); i != matches.end(); ++i) {
    const HomogenousPoint & point_from = (inverse ? points_to[i->first] : points_from[i->second]);
    const HomogenousPoint & point_to = (inverse ? points_from[i->second] : points_to[i->first]);

    HomogenousPoint p21 = h * point_from;
    p21 /= p21[2];
    precision_t chi_square2 =
        ((p21[0] - point_to[0]) * (p21[0] - point_to[0]) + (p21[1] - point_to[1]) * (p21[1] - point_to[1]))
            * sigma_squared_inv_;

    if (! (chi_square2 > HOMOGRAPHY_SCORE)) {
      out_inliers.insert(i->first);
      score += HOMOGRAPHY_SCORE - chi_square2;
    }
  }
  return score;
}

void HomographyMatrixEstimator::FindHomographyMatrix(const std::vector<HomogenousPoint> & points_to,
                                                     const std::vector<HomogenousPoint> & points_from,
                                                     const std::unordered_map<std::size_t, std::size_t> & matches,
                                                     const std::vector<size_t> & good_match_random_idx,
                                                     TMatrix33 & out_homography) {

  Eigen::Matrix<precision_t, Eigen::Dynamic, 9> L;
  L.resize(good_match_random_idx.size() * 2, Eigen::NoChange);

  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const auto f = matches.find(good_match_random_idx[i]);
    const auto & U = points_to[f->first];
    const auto & X = points_from[f->second];

    L(2 * i, 0) = X[0] * U[2];
    L(2 * i, 1) = X[1] * U[2];
    L(2 * i, 2) = X[2] * U[2];
    L(2 * i, 3) = 0;
    L(2 * i, 4) = 0;
    L(2 * i, 5) = 0;
    L(2 * i, 6) = -U[0] * X[0];
    L(2 * i, 7) = -U[0] * X[1];
    L(2 * i, 8) = -U[0] * X[2];

    L(2 * i + 1, 0) = 0;
    L(2 * i + 1, 1) = 0;
    L(2 * i + 1, 2) = 0;
    L(2 * i + 1, 3) = X[0] * U[2];
    L(2 * i + 1, 4) = X[1] * U[2];
    L(2 * i + 1, 5) = X[2] * U[2];
    L(2 * i + 1, 6) = -U[1] * X[0];
    L(2 * i + 1, 7) = -U[1] * X[1];
    L(2 * i + 1, 8) = -U[1] * X[2];
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

}
}
