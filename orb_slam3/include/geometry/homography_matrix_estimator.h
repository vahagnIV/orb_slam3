//
// Created by vahagn on 08/02/21.
//

#ifndef ORB_SLAM3_HOMOGRAPHY_MATRIX_ESTIMATOR_H
#define ORB_SLAM3_HOMOGRAPHY_MATRIX_ESTIMATOR_H
#include <typedefs.h>
#include <geometry/transfromation_estimator_base.h>
namespace orb_slam3 {
namespace geometry {

class HomographyMatrixEstimator : protected TransfromationEstimatorBase {
  typedef std::vector<std::pair<size_t, size_t>> pairs_t;
 public:
  HomographyMatrixEstimator(precision_t sigma) : TransfromationEstimatorBase(sigma) {}

  void FindBestHomographyMatrix(const std::vector<TPoint3D> & kp1,
                                const std::vector<TPoint3D> & kp2,
                                const std::vector<std::pair<size_t, size_t>> & good_matches,
                                const std::vector<std::vector<size_t>> & good_match_random_idx,
                                TMatrix33 & out_homography,
                                std::vector<bool> & out_inliers,
                                precision_t & out_error) const;

  precision_t ComputeHomographyReprojectionError(const TMatrix33 & h,
                                                 const std::vector<TPoint3D> & kp1,
                                                 const std::vector<TPoint3D> & kp2,
                                                 const pairs_t & good_matches,
                                                 std::vector<bool> & out_inliers,
                                                 bool inverse) const;

  void FindHomographyMatrix(const std::vector<TPoint3D> & kp1,
                            const std::vector<TPoint3D> & kp2,
                            const std::vector<std::pair<size_t, size_t>> & good_matches,
                            const std::vector<size_t> & good_match_random_idx,
                            TMatrix33 & out_homography) const;
 private:
  static const precision_t HOMOGRAPHY_THRESHOLD;

};

}
}
#endif //ORB_SLAM3_HOMOGRAPHY_MATRIX_ESTIMATOR_H
