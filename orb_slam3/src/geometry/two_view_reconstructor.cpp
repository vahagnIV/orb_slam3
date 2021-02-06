//
// Created by vahagn on 03/02/21.
//
#include <random>
#include <unordered_set>

#include <geometry/two_view_reconstructor.h>

namespace orb_slam3 {
namespace geometry {

const precision_t TwoViewReconstructor::FUNDAMENTAL_THRESHOLD = 3.841;
const precision_t TwoViewReconstructor::HOMOGRAPHY_THRESHOLD = 5.991;
const precision_t TwoViewReconstructor::FUNDAMENTAL_THRESHOLD_SCORE = 5.991;

TwoViewReconstructor::TwoViewReconstructor(const std::shared_ptr<camera::MonocularCamera> & left,
                                           const std::shared_ptr<camera::MonocularCamera> & right,
                                           const unsigned number_of_ransac_iterations,
                                           precision_t sigma_threshold)
    : left_(left),
      right_(right),
      number_of_ransac_iterations_(number_of_ransac_iterations),
      sigma_threshold_(sigma_threshold),
      sigma_squared_inv_(1 / sigma_threshold / sigma_threshold) {

}

void TwoViewReconstructor::Reconstruct(const std::vector<features::KeyPoint> & kp1,
                                       const std::vector<features::KeyPoint> & kp2,
                                       const std::vector<int> & matches12,
                                       TPose & out_pose,
                                       std::vector<TPoint3D> & out_points,
                                       std::vector<bool> & out_outliers,
                                       const size_t number_of_matches) const {
  std::vector<std::pair<size_t, size_t>> pre_matches;
  out_outliers.resize(matches12.size(), false);
  FilterGoodMatches(matches12, number_of_matches, pre_matches);
  std::vector<std::vector<size_t>> random_match_subset_idx;
  GenerateRandomSubsets(0, pre_matches.size(), 8, number_of_ransac_iterations_, random_match_subset_idx);
  precision_t h_error;
  precision_t f_error;
  TMatrix33 homography, fundamental;
  std::vector<bool> homography_inliers, fundamental_inliers;
  FindBestFundamentalMatrix(kp1, kp2, pre_matches, random_match_subset_idx, fundamental, fundamental_inliers, f_error);
  FindBestHomographyMatrix(kp1, kp2, pre_matches, random_match_subset_idx, homography, homography_inliers, h_error);
  if(f_error > h_error){

  }
  else{

  }


}

void TwoViewReconstructor::FindBestHomographyMatrix(const std::vector<features::KeyPoint> & kp1,
                                                    const std::vector<features::KeyPoint> & kp2,
                                                    const std::vector<std::pair<size_t, size_t>> & good_matches,
                                                    const std::vector<std::vector<size_t>> & good_match_random_idx,
                                                    TMatrix33 & out_homography,
                                                    std::vector<bool> & out_inliers,
                                                    precision_t & out_error) const {
  out_error = std::numeric_limits<precision_t>::max();
  TMatrix33 tmp_homography;
  std::vector<bool> tmp_inliers;
  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const std::vector<size_t> & good_matches_rnd = good_match_random_idx[i];
    FindHomographyMatrix(kp1, kp2, good_matches, good_matches_rnd, tmp_homography);
    precision_t error = ComputeHomographyReprojectionError(tmp_homography, kp1, kp2, good_matches, tmp_inliers);
    if (error > 0 && out_error > error) {
      out_homography = tmp_homography;
      out_inliers = tmp_inliers;
      out_error = error;
    }
  }
}

precision_t TwoViewReconstructor::ComputeHomographyReprojectionError(const TMatrix33 & h,
                                                                     const std::vector<features::KeyPoint> & kp1,
                                                                     const std::vector<features::KeyPoint> & kp2,
                                                                     const std::vector<std::pair<size_t,
                                                                                                 size_t>> & good_matches,
                                                                     std::vector<bool> & out_inliers) const {
  out_inliers.resize(good_matches.size(), true);
  std::fill(out_inliers.begin(), out_inliers.end(), true);
  precision_t error = 0;
  for (size_t i = 0; i < good_matches.size(); ++i) {
    const auto & match = good_matches[i];
    TPoint3D point_from;
    point_from << kp2[match.second].pt, 1;
    TPoint3D point_to;
    point_to << kp1[match.first].pt, 1;

    TPoint3D p21 = h * point_from, p12 = h.inverse() * point_to;
    p21 /= p21[2];
    p12 /= p12[2];

    precision_t chi_square1 =
        ((p12[0] - point_to[0]) * (p12[0] - point_to[0]) + (p12[1] - point_to[1]) * (p12[1] - point_to[1]))
            * sigma_squared_inv_;
    precision_t chi_square2 =
        ((p21[0] - point_from[0]) * (p21[0] - point_from[0]) + (p21[1] - point_from[1]) * (p21[1] - point_from[1]))
            * sigma_squared_inv_;
    if (chi_square1 > HOMOGRAPHY_THRESHOLD) {
      out_inliers[i] = false;
    } else
      error += HOMOGRAPHY_THRESHOLD - chi_square1;

    if (chi_square2 > HOMOGRAPHY_THRESHOLD) {
      out_inliers[i] = false;
    } else
      error += HOMOGRAPHY_THRESHOLD - chi_square2;

  }
  return error;
}

void TwoViewReconstructor::FindHomographyMatrix(const std::vector<features::KeyPoint> & kp1,
                                                const std::vector<features::KeyPoint> & kp2,
                                                const std::vector<std::pair<size_t, size_t>> & good_matches,
                                                const std::vector<size_t> & good_match_random_idx,
                                                TMatrix33 & out_homography) const {

  Eigen::Matrix<precision_t, Eigen::Dynamic, 9> L;
  L.resize(good_match_random_idx.size() * 2, Eigen::NoChange);

  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const auto & U = kp1[good_matches[good_match_random_idx[i]].first].pt;
    const auto & X = kp2[good_matches[good_match_random_idx[i]].second].pt;

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

precision_t TwoViewReconstructor::ComputeFundamentalReprojectionError(const TMatrix33 & f,
                                                                      const std::vector<features::KeyPoint> & kp1,
                                                                      const std::vector<features::KeyPoint> & kp2,
                                                                      const std::vector<std::pair<size_t,
                                                                                                  size_t>> & good_matches,
                                                                      std::vector<bool> & out_inliers) const {
  precision_t error = 0;
  out_inliers.resize(good_matches.size(), true);
  std::fill(out_inliers.begin(), out_inliers.end(), true);
  for (size_t i = 0; i < good_matches.size(); ++i) {
    const auto & match = good_matches[i];
    const TPoint2D & point_from = kp2[match.second].pt;
    const TPoint2D & point_to = kp1[match.second].pt;

    TPoint3D pt_from_3{point_from[0], point_from[1], 1};
    TPoint3D pt_to_3{point_to[0], point_to[1], 1};

    TPoint3D F_pt_from = f * pt_from_3;
    TPoint3D pt_to_F = pt_to_3.transpose() * f;

    const precision_t err = pt_to_3.dot(F_pt_from);;

    const precision_t
        chi_square1 = err / (F_pt_from[0] * F_pt_from[0] + F_pt_from[1] * F_pt_from[1]) * sigma_squared_inv_;
    const precision_t chi_square2 = err / (pt_to_F[0] * pt_to_F[0] + pt_to_F[1] * pt_to_F[1]) * sigma_squared_inv_;
    if (chi_square1 > FUNDAMENTAL_THRESHOLD)
      out_inliers[i] = false;
    else
      error += FUNDAMENTAL_THRESHOLD_SCORE - chi_square1;

    if (chi_square2 > FUNDAMENTAL_THRESHOLD)
      out_inliers[i] = false;
    else
      error += FUNDAMENTAL_THRESHOLD_SCORE - chi_square2;
  }
  return error;
}

void TwoViewReconstructor::FindFundamentalMatrix(const std::vector<features::KeyPoint> & kp1,
                                                 const std::vector<features::KeyPoint> & kp2,
                                                 const std::vector<std::pair<size_t, size_t>> & good_matches,
                                                 const std::vector<size_t> & good_match_random_idx,
                                                 TMatrix33 & out_fundamental) const {
  Eigen::Matrix<precision_t, Eigen::Dynamic, 9> L;
  L.resize(good_match_random_idx.size(), Eigen::NoChange);

  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const auto & U = kp1[good_matches[good_match_random_idx[i]].first].pt;
    const auto & X = kp2[good_matches[good_match_random_idx[i]].second].pt;

    // u2 = X[0]
    // v2 = X[1]
    ///u1 U[0]
    // v1 U[1]

    L(i, 0) = X[0] * U[0];
    L(i, 1) = X[0] * U[1];
    L(i, 2) = X[0];
    L(i, 3) = X[1] * U[0];
    L(i, 4) = X[1] * U[1];
    L(i, 5) = X[1];
    L(i, 6) = U[0];
    L(i, 7) = U[1];
    L(i, 8) = 1;

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

  out_fundamental << v(0, 8), v(1, 8), v(2, 8),
      v(3, 8), v(4, 8), v(5, 8),
      v(6, 8), v(7, 8), v(8, 8);
  out_fundamental /= out_fundamental(2, 2);

}

void TwoViewReconstructor::FindBestFundamentalMatrix(const std::vector<features::KeyPoint> & kp1,
                                                     const std::vector<features::KeyPoint> & kp2,
                                                     const std::vector<std::pair<size_t, size_t>> & good_matches,
                                                     const std::vector<std::vector<size_t>> & good_match_random_idx,
                                                     TMatrix33 & out_fundamental,
                                                     std::vector<bool> & out_inliers,
                                                     precision_t & out_error) const {
  out_error = std::numeric_limits<precision_t>::max();
  TMatrix33 tmp_fundamental;
  std::vector<bool> tmp_inliers;
  for (size_t i = 0; i < good_match_random_idx.size(); ++i) {
    const std::vector<size_t> & good_matches_rnd = good_match_random_idx[i];
    FindFundamentalMatrix(kp1, kp2, good_matches, good_matches_rnd, tmp_fundamental);
    precision_t error = ComputeFundamentalReprojectionError(tmp_fundamental, kp1, kp2, good_matches, tmp_inliers);
    if (error > 0 && out_error > error) {
      out_fundamental = tmp_fundamental;
      out_inliers = tmp_inliers;
      out_error = error;
    }
  }

}

void TwoViewReconstructor::GenerateRandomSubsets(const size_t min,
                                                 const size_t max,
                                                 const size_t count,
                                                 size_t subset_count,
                                                 std::vector<std::vector<size_t>> & out_result) const {
  out_result.resize(subset_count);
  do { GenerateRandomSubset(min, max, count, out_result[subset_count - 1]); }
  while (--subset_count);
}

void TwoViewReconstructor::FilterGoodMatches(const std::vector<int> & matches12,
                                             const size_t number_of_matches,
                                             std::vector<std::pair<size_t, size_t>> & out_good_matches) const {
  out_good_matches.resize(number_of_matches);
  size_t idx = 0;
  for (size_t i = 0; i < matches12.size(); ++i) {
    if (matches12[i] >= 0) {
      out_good_matches[idx].first = i;
      out_good_matches[idx++].second = static_cast<size_t >(matches12[i]);
    }
  }
}

void TwoViewReconstructor::GenerateRandomSubset(const size_t min,
                                                const size_t max,
                                                const size_t count,
                                                std::vector<size_t> & out_result) const {
  std::random_device rand_dev;
  std::mt19937 generator(rand_dev());
  std::uniform_int_distribution<size_t> distr(min, max-1);
  std::unordered_set<size_t> chosen;
  out_result.reserve(count);
  while (chosen.size() < count)
    chosen.insert(distr(generator));
  std::copy(chosen.begin(), chosen.end(), std::back_inserter(out_result));
}

}
}