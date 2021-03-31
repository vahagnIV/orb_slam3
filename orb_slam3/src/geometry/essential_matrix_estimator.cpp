//
// Created by vahagn on 08/02/21.
//

// == orb-slam3 ===
#include "geometry/essential_matrix_estimator.h"
#include "geometry/utils.h"

namespace orb_slam3 {
namespace geometry {

const precision_t EssentialMatrixEstimator::ESSENTIAL_THRESHOLD = 3.841;
const precision_t EssentialMatrixEstimator::ESSENTIAL_THRESHOLD_SCORE = 5.991;

bool EssentialMatrixEstimator::FindPose(const TMatrix33 & essential,
                                        const std::vector<HomogenousPoint> & points_to,
                                        const std::vector<HomogenousPoint> & points_from,
                                        const std::vector<features::Match> & matches,
                                        std::vector<bool> & out_inliers,
                                        std::vector<TPoint3D> & out_triangulated,
                                        Pose & out_pose) const {
  // Hartley - Zisserman (page 258, 259)
  Eigen::JacobiSVD<TMatrix33> svd(essential, Eigen::ComputeFullV | Eigen::ComputeFullU);
  const TMatrix33 & U = svd.matrixU();
  const TMatrix33 VT = svd.matrixV().transpose();
  TMatrix33 W;
  W << 0, -1, 0,
      1, 0, 0,
      0, 0, 1;
  std::vector<Pose> candidate_solutions(4);
  TMatrix33 R1 = U * W * VT;
  if (R1.determinant() < 0) {
    R1 = -R1;
  }
  TMatrix33 R2 = U * W.transpose() * VT;
  if (R2.determinant() < 0) {
    R2 = -R2;
  }
  TVector3D T = U.col(2);

  candidate_solutions[0].R = R1;
  candidate_solutions[0].T = T;
  candidate_solutions[1].R = R1;
  candidate_solutions[1].T = -T;
  candidate_solutions[2].R = R2;
  candidate_solutions[2].T = T;
  candidate_solutions[3].R = R2;
  candidate_solutions[3].T = -T;
  return this->FindCorrectPose(candidate_solutions, points_to, points_from, matches, out_inliers,
                               out_triangulated, out_pose);

}

precision_t EssentialMatrixEstimator::ComputeEssentialReprojectionError(const TMatrix33 & E,
                                                                        const std::vector<HomogenousPoint> & points_to,
                                                                        const std::vector<HomogenousPoint> & points_from,
                                                                        const std::vector<features::Match> & matches,
                                                                        std::vector<bool> & out_inliers) const {
  precision_t score = 0;
  out_inliers.resize(matches.size(), true);
  std::fill(out_inliers.begin(), out_inliers.end(), true);
  for (size_t i = 0; i < matches.size(); ++i) {
    const auto & match = matches[i];

    const HomogenousPoint & point_from = points_from[match.from_idx];
    const HomogenousPoint & point_to = points_to[match.to_idx];

    HomogenousPoint f_from = E * point_from;
    HomogenousPoint to_f = point_to.transpose() * E;

    precision_t err = point_to.dot(f_from);

    err *= err;

    const precision_t chi_square1 = err / (f_from[0] * f_from[0] + f_from[1] * f_from[1]) * sigma_squared_inv_;

    const precision_t chi_square2 = err / (to_f[0] * to_f[0] + to_f[1] * to_f[1]) * sigma_squared_inv_;

    if (chi_square1 > ESSENTIAL_THRESHOLD || chi_square2 > ESSENTIAL_THRESHOLD) {
      out_inliers[i] = false;
      continue;
    }
    score += ESSENTIAL_THRESHOLD_SCORE - chi_square1;
    score += ESSENTIAL_THRESHOLD_SCORE - chi_square2;
  }
  return score;
}

void EssentialMatrixEstimator::FindEssentialMatrix(const std::vector<HomogenousPoint> & points_to,
                                                   const std::vector<HomogenousPoint> & points_from,
                                                   const std::vector<features::Match> & matches,
                                                   const std::vector<size_t> & good_match_random_idx,
                                                   TMatrix33 & out_essential) {
  Eigen::Matrix<precision_t, Eigen::Dynamic, 9> L;
  L.resize(good_match_random_idx.size(), Eigen::NoChange);

  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const auto & X = points_to[matches[good_match_random_idx[i]].to_idx];
    const auto & U = points_from[matches[good_match_random_idx[i]].from_idx];

    L(i, 0) = X[0] * U[0];
    L(i, 1) = X[0] * U[1];
    L(i, 2) = X[0] * U[2];
    L(i, 3) = X[1] * U[0];
    L(i, 4) = X[1] * U[1];
    L(i, 5) = X[1] * U[2];
    L(i, 6) = X[2] * U[0];
    L(i, 7) = X[2] * U[1];
    L(i, 8) = X[2] * U[2];

  }

  Eigen::JacobiSVD<Eigen::Matrix<precision_t, Eigen::Dynamic, 9>> svd(L, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (!svd.computeV())
    return;

  const Eigen::Matrix<precision_t, 9, 9> & v = svd.matrixV();

//  Eigen::JacobiSVD<Eigen::Matrix<precision_t, Eigen::Dynamic, 9>> svd2(v, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (!svd.computeV())
    return;
//  auto S = svd2.singularValues();
//  S[2] = 0;

  out_essential << v(0, 8), v(1, 8), v(2, 8),
      v(3, 8), v(4, 8), v(5, 8),
      v(6, 8), v(7, 8), v(8, 8);
  out_essential /= out_essential(2, 2);

}

void EssentialMatrixEstimator::FindBestEssentialMatrix(const std::vector<HomogenousPoint> & points_to,
                                                       const std::vector<HomogenousPoint> & points_from,
                                                       const std::vector<features::Match> & matches,
                                                       const std::vector<std::vector<size_t>> & good_match_random_idx,
                                                       TMatrix33 & out_essential,
                                                       std::vector<bool> & out_inliers,
                                                       precision_t & out_error) const {
  out_error = 0;//std::numeric_limits<precision_t>::max();
  TMatrix33 tmp_essential;
  std::vector<bool> tmp_inliers;
  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const std::vector<size_t> & good_matches_rnd = good_match_random_idx[i];
    FindEssentialMatrix(points_to, points_from, matches, good_matches_rnd, tmp_essential);
    precision_t
        error = ComputeEssentialReprojectionError(tmp_essential, points_to, points_from, matches, tmp_inliers);
    if (error > 0 && out_error < error) {
      out_essential = tmp_essential;
      out_inliers = tmp_inliers;
      out_error = error;
    }
  }

}

TMatrix33 EssentialMatrixEstimator::FromEuclideanTransformations(const TMatrix33 & R, const TVector3D & T) {
  return R * geometry::utils::SkewSymmetricMatrix(R.transpose() * T);
}

}
}