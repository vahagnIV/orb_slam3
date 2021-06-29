//
// Created by vahagn on 22/06/2021.
//

#ifndef ORB_SLAM3_SRC_GEOMETRY_SIM_3_SOLVER_H_
#define ORB_SLAM3_SRC_GEOMETRY_SIM_3_SOLVER_H_

#include "typedefs.h"
#include "pose.h"
namespace orb_slam3 {
namespace camera{
class MonocularCamera;
}

namespace geometry {

class Sim3Solver {
 public:
  Sim3Solver(){};

  static Pose EstimateSim3(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches,
                    const std::vector<std::pair<TPoint2D, TPoint2D>> & projections,
                    const camera::MonocularCamera * camera1,
                    const camera::MonocularCamera * camera2,
                    const std::vector<std::pair<precision_t, precision_t>> & errors,
                    size_t ransac_iteration_count);

  static Pose ComputeSim3(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches) ;
 private:
  static void ComputeCentroids(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches,
                        TPoint3D & out_centroid1, TPoint3D & out_centroid2) ;
  static void ComputeRelativeCoordinates(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches,
                                  const TPoint3D & centroid1,
                                  const TPoint3D & centroid2,
                                  std::vector<std::pair<TPoint3D, TPoint3D>> & out_relative_coords) ;
  static TMatrix33 ComputeM(const std::vector<std::pair<TPoint3D, TPoint3D>> & relative_coords) ;
  static TMatrix33 ComputeRotation(const std::vector<std::pair<TPoint3D, TPoint3D>> & relative_coords);
  static precision_t ComputeScale(const std::vector<std::pair<TPoint3D, TPoint3D>> & relative_coords, const TMatrix33 & R);
  static TVector3D ComputeTranslation(const TMatrix33 & R,
                                      const TVector3D & centroid1,
                                      const TVector3D & centroid2,
                                      precision_t s);

};

}
}
#endif //ORB_SLAM3_SRC_GEOMETRY_SIM_3_SOLVER_H_
