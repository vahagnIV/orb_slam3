//
// Created by vahagn on 08/02/21.
//

#ifndef ORB_SLAM3_HOMOGRAPHY_MATRIX_ESTIMATOR_H
#define ORB_SLAM3_HOMOGRAPHY_MATRIX_ESTIMATOR_H

// == orb-slam3 ===
#include <typedefs.h>
#include <geometry/transfromation_estimator_base.h>
#include <geometry/pose.h>
#include <features/match.h>
namespace orb_slam3 {
namespace geometry {

class HomographyMatrixEstimator : protected TransfromationEstimatorBase {
  typedef struct { TMatrix33 R; TVector3D T; } Solution;
 public:
  HomographyMatrixEstimator(precision_t sigma) : TransfromationEstimatorBase(sigma) {}

  void FindBestHomographyMatrix(const std::vector<HomogenousPoint> & points_to,
                                const std::vector<HomogenousPoint> & points_from,
                                const std::vector<features::Match> & matches,
                                const std::vector<std::vector<size_t>> & good_match_random_idx,
                                TMatrix33 & out_homography,
                                std::vector<bool> & out_inliers,
                                precision_t & out_score) const;

  precision_t ComputeHomographyReprojectionError(const TMatrix33 & h,
                                                 const std::vector<HomogenousPoint> & points_to,
                                                 const std::vector<HomogenousPoint> & points_from,
                                                 const std::vector<features::Match> & matches,
                                                 std::vector<bool> & out_inliers,
                                                 bool inverse) const;

  void FindHomographyMatrix(const std::vector<HomogenousPoint> & points_to,
                            const std::vector<HomogenousPoint> & points_from,
                            const std::vector<features::Match> & matches,
                            const std::vector<size_t> & good_match_random_idx,
                            TMatrix33 & out_homography) const;

  /*!
   * Finds the rotation and translation from homography and matched points
   * @param homography An estimated homography for a plane in the scene.
   * @param points_to The points in the r.h.s. coordinate system  of the homogrpahy
   * @param points_from The points in the l.h.s. coordinate system  of the homogrpahy
   * @param matches The matches between the poinnts to and from
   * @param out_inliers output vector of inliers, i.e. matches that passed the rigidity constraints
   * @param out_triangulated The triangulated points in the "from" coordinate system
   * @param out_rotation The rotation matrix from points_from to points_to
   * @param out_translation The translation vector from points_from to points_to
   * @return
   */
  bool FindRTTransformation(const TMatrix33 & homography,
                            const std::vector<TPoint3D> & points_to,
                            const std::vector<TPoint3D> & points_from,
                            const std::vector<features::Match> & matches,
                            std::vector<bool> & out_inliers,
                            std::vector<TPoint3D> & out_triangulated,
                            TMatrix33 & out_rotation,
                            TVector3D & out_translation) const;

  size_t CheckRT(const Solution & solution,
                 const std::vector<HomogenousPoint> & points_to,
                 const std::vector<HomogenousPoint> & points_from,
                 const std::vector<features::Match> & matches,
                 std::vector<bool> & inliers,
                 precision_t & out_parallax,
                 std::vector<TPoint3D> & out_triangulated) const;

  bool Triangulate(const Solution & sol,
                   const HomogenousPoint & point_from,
                   const HomogenousPoint & point_to,
                   TPoint3D & out_trinagulated) const;
 private:

  void FillSolutionsForPositiveD(precision_t d1,
                                 precision_t d2,
                                 precision_t d3,
                                 const TMatrix33 & U,
                                 const TMatrix33 & VT,
                                 Solution solution[4],
                                 precision_t s) const noexcept;

  void FillSolutionsForNegativeD(precision_t d1,
                                 precision_t d2,
                                 precision_t d3,
                                 const TMatrix33 & U,
                                 const TMatrix33 & VT,
                                 Solution solution[4],
                                 precision_t s) const noexcept;


  precision_t ComputeParallax(const TPoint3D & point, const Solution & solution) const;

  precision_t ComputeTriangulatedReprojectionError(const TPoint3D & point, const HomogenousPoint & original_point) const;

  static const precision_t HOMOGRAPHY_SCORE;
  static const precision_t PARALLAX_THRESHOLD;

};

}
}
#endif //ORB_SLAM3_HOMOGRAPHY_MATRIX_ESTIMATOR_H
