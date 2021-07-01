//
// Created by vahagn on 30/06/2021.
//

#ifndef ORB_SLAM3_SRC_GEOMETRY_RANSAC_SIM_3_SOLVER_H_
#define ORB_SLAM3_SRC_GEOMETRY_RANSAC_SIM_3_SOLVER_H_

#include "typedefs.h"
#include "sim3_solver.h"
#include "random_subset_generator.h"

namespace orb_slam3 {

namespace camera{
class MonocularCamera;
}

namespace geometry {

class RANSACSim3Solver {
 public:
  RANSACSim3Solver(const std::vector<std::pair<TPoint3D, TPoint3D>> * matches,
                   const std::vector<std::pair<TPoint2D, TPoint2D>> * projections,
                   const camera::MonocularCamera * camera1,
                   const camera::MonocularCamera * camera2,
                   const std::vector<std::pair<precision_t, precision_t>> * errors,
                   size_t ransac_iteration_count,
                   size_t min_inliers_count = 15);
  bool operator()(Pose & out_pose);
 private:
  /*!
   * Checks the rigidity constraint
   * @return number of inliers
   */
  size_t CheckPose(const Pose & pose);
 private:
  static const size_t MIN_NUMBER_OF_MATCHES_DEFAULT;
  const std::vector<std::pair<TPoint3D, TPoint3D>> & matches_;
  const std::vector<std::pair<TPoint2D, TPoint2D>> & projections_;
  const camera::MonocularCamera * camera1_;
  const camera::MonocularCamera * camera2_;
  const std::vector<std::pair<precision_t, precision_t>> & errors_;
  const size_t ransac_iteration_count_;
  const size_t min_inliers_count_;
  RandomSubsetGenerator subset_generator_;
};

}
}

#endif //ORB_SLAM3_SRC_GEOMETRY_RANSAC_SIM_3_SOLVER_H_
