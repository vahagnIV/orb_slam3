//
// Created by vahagn on 03/02/21.
//

#ifndef ORB_SLAM3_TWO_VIEW_RECONSTRUCTOR_H
#define ORB_SLAM3_TWO_VIEW_RECONSTRUCTOR_H

// == stl ===
#include <memory>

// == orb-slam3 ===
#include <camera/monocular_camera.h>
#include <geometry/essential_matrix_estimator.h>
#include <geometry/homography_matrix_estimator.h>
#include <features/match.h>

namespace orb_slam3 {
namespace geometry {

class TwoViewReconstructor {
 public:
  TwoViewReconstructor(unsigned number_of_ransac_iterations,
                       precision_t sigma_threshold = 1.0);

  bool Reconstruct(const std::vector<HomogenousPoint> & points_to,
                   const std::vector<HomogenousPoint> & points_from,
                   const std::unordered_map<std::size_t, std::size_t> & matches,
                   Pose & out_pose,
                   std::unordered_map<std::size_t, TPoint3D> & out_points) const;
 private:

  static void GenerateRandomSubset(size_t min,
                            size_t max,
                            size_t count,
                            std::vector<size_t> & out_result) ;

  static void GenerateRandomSubsets(size_t min,
                             size_t max,
                             size_t count,
                             size_t subset_count,
                             const std::unordered_map<std::size_t, std::size_t> & matches,
                             std::vector<std::vector<size_t>> & out_result) ;

 private:
  const unsigned number_of_ransac_iterations_;
  EssentialMatrixEstimator essential_matrix_estimator_;
  HomographyMatrixEstimator homography_matrix_estimator_;



};

}
}
#endif //ORB_SLAM3_TWO_VIEW_RECONSTRUCTOR_H
