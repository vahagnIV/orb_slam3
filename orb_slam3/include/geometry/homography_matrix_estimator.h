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

 public:
  explicit HomographyMatrixEstimator(precision_t sigma) : TransfromationEstimatorBase(sigma, 0.75) {}

  void FindBestHomographyMatrix(const std::vector<HomogenousPoint> & points_to,
                                const std::vector<HomogenousPoint> & points_from,
                                const std::unordered_map<std::size_t, std::size_t> & matches,
                                const std::vector<std::vector<size_t>> & good_match_random_idx,
                                TMatrix33 & out_homography,
                                std::unordered_set<std::size_t> & out_inliers,
                                precision_t & out_score) const;

  precision_t ComputeHomographyReprojectionError(const TMatrix33 & h,
                                                 const std::vector<HomogenousPoint> & points_to,
                                                 const std::vector<HomogenousPoint> & points_from,
                                                 const std::unordered_map<std::size_t, std::size_t> & matches,
                                                 std::unordered_set<std::size_t> & out_inliers,
                                                 bool inverse) const;

  static void FindHomographyMatrix(const std::vector<HomogenousPoint> & points_to,
                                   const std::vector<HomogenousPoint> & points_from,
                                   const std::unordered_map<std::size_t, std::size_t> & matches,
                                   const std::vector<size_t> & good_match_random_idx,
                                   TMatrix33 & out_homography);

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
  bool FindPose(const TMatrix33 & homography,
                const std::vector<HomogenousPoint> & points_to,
                const std::vector<HomogenousPoint> & points_from,
                const std::unordered_map<std::size_t, std::size_t> & matches,
                std::unordered_set<size_t> & out_inliers,
                std::unordered_map<std::size_t, TPoint3D> & out_triangulated,
                Pose & out_pose) const;

 private:

  static void FillSolutionsForPositiveD(precision_t d1,
                                 precision_t d2,
                                 precision_t d3,
                                 const TMatrix33 & U,
                                 const TMatrix33 & VT,
                                 std::vector<Pose>::iterator start,
                                 std::vector<Pose>::iterator end,
                                 precision_t s) noexcept;

  static void FillSolutionsForNegativeD(precision_t d1,
                                        precision_t d2,
                                        precision_t d3,
                                        const TMatrix33 & U,
                                        const TMatrix33 & VT,
                                        std::vector<Pose>::iterator start,
                                        std::vector<Pose>::iterator end,
                                        precision_t s) noexcept;

  static const precision_t HOMOGRAPHY_SCORE;

};

}
}
#endif //ORB_SLAM3_HOMOGRAPHY_MATRIX_ESTIMATOR_H
