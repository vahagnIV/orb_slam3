//
// Created by vahagn on 08/02/21.
//

#ifndef ORB_SLAM3_FUNDAMENTAL_MATRIX_ESTIMATOR_H
#define ORB_SLAM3_FUNDAMENTAL_MATRIX_ESTIMATOR_H

// == orb-slam3 ===
#include <geometry/transfromation_estimator_base.h>
#include <features/match.h>

namespace orb_slam3 {
namespace geometry {

class FundamentalMatrixEstimator : protected TransfromationEstimatorBase {
 public:
  FundamentalMatrixEstimator(precision_t sigma) : TransfromationEstimatorBase(sigma) {}

  precision_t ComputeFundamentalReprojectionError(const TMatrix33 &f,
                                                  const std::vector<HomogenousPoint> &kp1,
                                                  const std::vector<HomogenousPoint> &kp2,
                                                  const std::vector<features::Match> &matches,
                                                  std::vector<bool> &out_inliers) const;

  static void FindFundamentalMatrix(const std::vector<HomogenousPoint> &kp1,
                                    const std::vector<HomogenousPoint> &kp2,
                                    const std::vector<features::Match> &matches,
                                    const std::vector<size_t> &good_match_random_idx,
                                    TMatrix33 &out_fundamental);

  void FindBestFundamentalMatrix(const std::vector<HomogenousPoint> &kp1,
                                 const std::vector<HomogenousPoint> &kp2,
                                 const std::vector<features::Match> &matches,
                                 const std::vector<std::vector<size_t>> &good_match_random_idx,
                                 TMatrix33 &out_fundamental,
                                 std::vector<bool> &out_inliers,
                                 precision_t &out_error) const;

  bool FindPose(const TMatrix33 &essential,
                const std::vector<HomogenousPoint> &points_to,
                const std::vector<HomogenousPoint> &points_from,
                const std::vector<features::Match> &matches,
                std::vector<bool> &out_inliers,
                std::vector<TPoint3D> &out_triangulated,
                Pose &out_pose) const;

  static TMatrix33 FromEuclideanTransformations(const TMatrix33 &R, const TVector3D &T);

 private:
  static const precision_t FUNDAMENTAL_THRESHOLD;
  static const precision_t FUNDAMENTAL_THRESHOLD_SCORE;

};

}
}
#endif //ORB_SLAM3_FUNDAMENTAL_MATRIX_ESTIMATOR_H
