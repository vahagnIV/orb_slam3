//
// Created by vahagn on 08/02/21.
//

#ifndef ORB_SLAM3_ESSENTIAL_MATRIX_ESTIMATOR_H
#define ORB_SLAM3_ESSENTIAL_MATRIX_ESTIMATOR_H

// == orb-slam3 ===
#include "transfromation_estimator_base.h"
#include "../features/match.h"

namespace orb_slam3 {
namespace geometry {

class EssentialMatrixEstimator : protected TransfromationEstimatorBase {
 public:
  EssentialMatrixEstimator(precision_t sigma) : TransfromationEstimatorBase(sigma, 0.7) {}

  precision_t ComputeEssentialReprojectionError(const TMatrix33 & E,
                                                const std::vector<HomogenousPoint> & points_to,
                                                const std::vector<HomogenousPoint> & points_from,
                                                const std::unordered_map<std::size_t, std::size_t> & matches,
                                                std::unordered_set<std::size_t> & out_inliers) const;

  static void FindEssentialMatrix(const std::vector<HomogenousPoint> & points_to,
                                  const std::vector<HomogenousPoint> & points_from,
                                  const std::unordered_map<std::size_t, std::size_t> & matches,
                                  const std::vector<size_t> & good_match_random_idx,
                                  TMatrix33 & out_essential);

  void FindBestEssentialMatrix(const std::vector<HomogenousPoint> & points_to,
                               const std::vector<HomogenousPoint> & points_from,
                               const std::unordered_map<std::size_t, std::size_t> & matches,
                               const std::vector<std::vector<size_t>> & good_match_random_idx,
                               TMatrix33 & out_essential,
                               std::unordered_set<std::size_t> & out_inliers,
                               precision_t & out_score) const;

  bool FindPose(const TMatrix33 & essential,
                const std::vector<HomogenousPoint> & points_to,
                const std::vector<HomogenousPoint> & points_from,
                const std::unordered_map<std::size_t, std::size_t> & matches,
                std::unordered_map<std::size_t, TPoint3D> & out_triangulated,
                Pose & out_pose) const;

  static TMatrix33 FromEuclideanTransformations(const TMatrix33 & R, const TVector3D & T);
 protected:
  static void NormalizePoints(const std::vector<HomogenousPoint> & points,
                       std::vector<HomogenousPoint> & out_normalized_points,
                       TMatrix33 & out_statistical_matrix) ;

 private:
  static const precision_t ESSENTIAL_THRESHOLD;
  static const precision_t ESSENTIAL_THRESHOLD_SCORE;

};

}
}
#endif //ORB_SLAM3_ESSENTIAL_MATRIX_ESTIMATOR_H
