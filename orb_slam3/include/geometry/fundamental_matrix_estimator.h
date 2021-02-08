//
// Created by vahagn on 08/02/21.
//

#ifndef ORB_SLAM3_FUNDAMENTAL_MATRIX_ESTIMATOR_H
#define ORB_SLAM3_FUNDAMENTAL_MATRIX_ESTIMATOR_H
#include <geometry/transfromation_estimator_base.h>

namespace orb_slam3 {
namespace geometry {

class FundamentalMatrixEstimator: protected TransfromationEstimatorBase {
  typedef std::vector<std::pair<size_t, size_t>> pairs_t;
 public:
  FundamentalMatrixEstimator(precision_t sigma): TransfromationEstimatorBase(sigma){}

  precision_t ComputeFundamentalReprojectionError(const TMatrix33 & homography,
                                                  const std::vector<TPoint2D> & kp1,
                                                  const std::vector<TPoint2D> & kp2,
                                                  const std::vector<std::pair<size_t, size_t>> & good_matches,
                                                  std::vector<bool> & out_inliers) const;

  void FindFundamentalMatrix(const std::vector<TPoint2D> & kp1,
                             const std::vector<TPoint2D> & kp2,
                             const pairs_t & good_matches,
                             const std::vector<size_t> & good_match_random_idx,
                             TMatrix33 & out_fundamental) const;

  void FindBestFundamentalMatrix(const std::vector<TPoint2D> & kp1,
                                 const std::vector<TPoint2D> & kp2,
                                 const pairs_t & good_matches,
                                 const std::vector<std::vector<size_t>> & good_match_random_idx,
                                 TMatrix33 & out_fundamental,
                                 std::vector<bool> & out_inliers,
                                 precision_t & out_error) const;

 private:
  static const precision_t FUNDAMENTAL_THRESHOLD;
  static const precision_t FUNDAMENTAL_THRESHOLD_SCORE;

};

}
}
#endif //ORB_SLAM3_FUNDAMENTAL_MATRIX_ESTIMATOR_H
