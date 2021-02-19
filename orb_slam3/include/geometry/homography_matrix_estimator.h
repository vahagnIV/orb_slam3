//
// Created by vahagn on 08/02/21.
//

#ifndef ORB_SLAM3_HOMOGRAPHY_MATRIX_ESTIMATOR_H
#define ORB_SLAM3_HOMOGRAPHY_MATRIX_ESTIMATOR_H
#include <typedefs.h>
#include <geometry/transfromation_estimator_base.h>
#include <geometry/pose.h>
namespace orb_slam3 {
namespace geometry {

class HomographyMatrixEstimator : protected TransfromationEstimatorBase {
  typedef std::vector<std::pair<size_t, size_t>> pairs_t;
 public:
  HomographyMatrixEstimator(precision_t sigma) : TransfromationEstimatorBase(sigma) {}

  void FindBestHomographyMatrix(const std::vector<HomogenousPoint> & points_to,
                                const std::vector<HomogenousPoint> & points_from,
                                const pairs_t & good_matches,
                                const std::vector<std::vector<size_t>> & good_match_random_idx,
                                TMatrix33 & out_homography,
                                std::vector<bool> & out_inliers,
                                precision_t & out_score) const;

  precision_t ComputeHomographyReprojectionError(const TMatrix33 & h,
                                                 const std::vector<HomogenousPoint> & points_to,
                                                 const std::vector<HomogenousPoint> & points_from,
                                                 const pairs_t & good_matches,
                                                 std::vector<bool> & out_inliers,
                                                 bool inverse) const;

  void FindHomographyMatrix(const std::vector<HomogenousPoint> & points_to,
                            const std::vector<HomogenousPoint> & points_from,
                            const std::vector<std::pair<size_t, size_t>> & good_matches,
                            const std::vector<size_t> & good_match_random_idx,
                            TMatrix33 & out_homography) const;

  bool FindRTTransformation(const TMatrix33 & homography,
                            const std::vector<TPoint3D> & points_to,
                            const std::vector<TPoint3D> & points_from,
                            const pairs_t & good_matches,
                            std::vector<bool> & out_inliers,
                            std::vector<TPoint3D> & out_triangulated,
                            geometry::Pose & out_pose) const;

  size_t CheckRT(const geometry::Pose & solution,
                 const std::vector<HomogenousPoint> & points_to,
                 const std::vector<HomogenousPoint> & points_from,
                 const pairs_t & good_matches,
                 std::vector<bool> & inliers,
                 precision_t & out_parallax,
                 std::vector<TPoint3D> & out_triangulated) const;

  bool Triangulate(const geometry::Pose & sol,
                   const HomogenousPoint & point_to,
                   const HomogenousPoint & point_from,
                   TPoint3D & out_trinagulated) const;
 private:

  void FillSolutionsForPositiveD(precision_t d1,
                                 precision_t d2,
                                 precision_t d3,
                                 const TMatrix33 & U,
                                 const TMatrix33 & VT,
                                 geometry::Pose solution[4],
                                 precision_t s) const noexcept;

  void FillSolutionsForNegativeD(precision_t d1,
                                 precision_t d2,
                                 precision_t d3,
                                 const TMatrix33 & U,
                                 const TMatrix33 & VT,
                                 geometry::Pose solution[4],
                                 precision_t s) const noexcept;


  precision_t ComputeParallax(const TPoint3D & point, const geometry::Pose & solution) const;

  precision_t ComputeTriangulatedReprojectionError(const TPoint3D & point, const HomogenousPoint & original_point) const;

  static const precision_t HOMOGRAPHY_SCORE;
  static const precision_t PARALLAX_THRESHOLD;

};

}
}
#endif //ORB_SLAM3_HOMOGRAPHY_MATRIX_ESTIMATOR_H
