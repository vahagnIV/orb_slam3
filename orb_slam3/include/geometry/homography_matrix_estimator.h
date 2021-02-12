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
  typedef struct {
    TMatrix33 R;
    TVector3D T;
    TVector3D n;
  } Solution;
 public:
  HomographyMatrixEstimator(precision_t sigma) : TransfromationEstimatorBase(sigma) {}

  void FindBestHomographyMatrix(const std::vector<TPoint3D> & kp1,
                                const std::vector<TPoint3D> & kp2,
                                const pairs_t & good_matches,
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

  bool FindRTTransformation(const TMatrix33 & homography,
                            const std::vector<TPoint3D> & kp1,
                            const std::vector<TPoint3D> & kp2,
                            const pairs_t & good_matches,
                            const std::vector<bool> & out_inliers,
                            std::vector<TPoint3D> & out_triangulated,
                            TPose & out_pose) const;

  int CheckRT(const Solution & solution,
              const std::vector<TPoint3D> & kp1,
              const std::vector<TPoint3D> & kp2,
              const pairs_t & good_matches,
              const std::vector<bool> & inliers,
              std::vector<TPoint3D> & trinagulated) const;
 private:

  void FillSolutionsForPositiveD(precision_t d1,
                                 precision_t d2,
                                 precision_t d3,
                                 const TMatrix33 & U,
                                 const TMatrix33 & VT,
                                 Solution solution[4],
                                 precision_t s) const;
  void FillSolutionsForNegativeD(precision_t d1,
                                 precision_t d2,
                                 precision_t d3,
                                 const TMatrix33 & U,
                                 const TMatrix33 & VT,
                                 Solution solution[4],
                                 precision_t s) const;

  static const precision_t HOMOGRAPHY_THRESHOLD;

};

}
}
#endif //ORB_SLAM3_HOMOGRAPHY_MATRIX_ESTIMATOR_H
