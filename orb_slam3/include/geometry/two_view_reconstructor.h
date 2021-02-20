//
// Created by vahagn on 03/02/21.
//

#ifndef ORB_SLAM3_TWO_VIEW_RECONSTRUCTOR_H
#define ORB_SLAM3_TWO_VIEW_RECONSTRUCTOR_H

// == stl ===
#include <memory>

// == orb-slam3 ===
#include <camera/monocular_camera.h>
#include <geometry/fundamental_matrix_estimator.h>
#include <geometry/homography_matrix_estimator.h>
#include <features/match.h>

namespace orb_slam3 {
namespace geometry {

class TwoViewReconstructor {
 public:
  TwoViewReconstructor(const std::shared_ptr<camera::MonocularCamera> & left,
                       const std::shared_ptr<camera::MonocularCamera> & right,
                       const unsigned number_of_ransac_iterations,
                       precision_t sigma_threshold = 1.0);

  bool Reconstruct(const std::vector<HomogenousPoint> & points_to,
                   const std::vector<HomogenousPoint> & points_from,
                   const std::vector<features::Match> & matches,
                   Pose & out_pose,
                   std::vector<TPoint3D> & out_points,
                   std::vector<bool> & out_outliers) const;
 private:

  void GenerateRandomSubset(size_t min,
                            size_t max,
                            size_t count,
                            std::vector<size_t> & out_result) const;

  void GenerateRandomSubsets(size_t min,
                             size_t max,
                             size_t count,
                             size_t subset_count,
                             std::vector<std::vector<size_t>> & out_result) const;

 private:
  const std::shared_ptr<camera::MonocularCamera> left_, right_;
  const unsigned number_of_ransac_iterations_;
  FundamentalMatrixEstimator fundamental_matrix_estimator_;
  HomographyMatrixEstimator homography_matrix_sstimator_;



};

}
}
#endif //ORB_SLAM3_TWO_VIEW_RECONSTRUCTOR_H
