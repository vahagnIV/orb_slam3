//
// Created by vahagn on 30/06/2021.
//

#include "ransac_sim3_solver.h"
#include <camera/monocular_camera.h>

namespace orb_slam3 {
namespace geometry {

const size_t RANSACSim3Solver::MIN_NUMBER_OF_MATCHES_DEFAULT = 10;

RANSACSim3Solver::RANSACSim3Solver(const std::vector<std::pair<TPoint3D, TPoint3D>> * matches,
                                   const std::vector<std::pair<TPoint2D, TPoint2D>> * projections,
                                   const camera::MonocularCamera * camera1,
                                   const camera::MonocularCamera * camera2,
                                   const std::vector<std::pair<precision_t, precision_t>> * errors,
                                   size_t ransac_iteration_count,
                                   size_t min_inliers_count) : matches_(*matches),
                                                               projections_(*projections),
                                                               camera1_(camera1),
                                                               camera2_(camera2),
                                                               errors_(*errors),
                                                               ransac_iteration_count_(ransac_iteration_count),
                                                               min_inliers_count_(min_inliers_count),
                                                               subset_generator_(MIN_NUMBER_OF_MATCHES_DEFAULT,
                                                                                 0,
                                                                                 matches->size()) {
  if (matches_.size() != projections_.size() || errors_.size() != matches_.size())
    throw std::runtime_error("RANSACSim3Solver: Assertion failed matches.size() == projections.size() == errors.size()");

}

bool RANSACSim3Solver::operator()(Sim3Transformation & out_pose) {
  for (size_t i = 0; i < ransac_iteration_count_; ++i) {
    std::vector<size_t> slice;
    subset_generator_.Generate(slice);
    out_pose = Sim3Solver::ComputeSim3(matches_, slice);
    size_t n_inliers = CheckPose(out_pose);
    if (n_inliers > min_inliers_count_) {
//      std::cout << "Found " << n_inliers << " inliers out_of " << matches_.size() << std::endl;
      return true;
    }
  }
  return false;
}

size_t RANSACSim3Solver::CheckPose(const Sim3Transformation & pose) {
  size_t number_of_inliers = 0;
  Sim3Transformation inverse_pose = pose.GetInverse();
  assert(matches_.size() == projections_.size() && matches_.size() == errors_.size());
  for (size_t i = 0; i < matches_.size(); ++i) {

    const auto & match = matches_[i];
    const auto & projection = projections_[i];
    const auto & error = errors_[i];

    TPoint3D local_pt1 = pose.Transform(match.second);
    TPoint3D local_pt2 = inverse_pose.Transform(match.first);

    TPoint2D proj1, proj2;
    camera1_->ProjectAndDistort(local_pt1, proj1);
    camera2_->ProjectAndDistort(local_pt2, proj2);

    if ((proj1 - projection.first).squaredNorm() < error.first
        && (proj2 - projection.second).squaredNorm() < error.second)
      ++number_of_inliers;
  }
  return number_of_inliers;

}

}
}