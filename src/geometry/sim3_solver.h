//
// Created by vahagn on 22/06/2021.
//

#ifndef ORB_SLAM3_SRC_GEOMETRY_SIM_3_SOLVER_H_
#define ORB_SLAM3_SRC_GEOMETRY_SIM_3_SOLVER_H_

#include <unordered_set>
#include "typedefs.h"
#include "pose.h"
namespace orb_slam3 {
namespace geometry {

class Sim3Solver {
 public:
  Sim3Solver(){};

  static Pose ComputeSim3(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches,
                          const std::vector<size_t> & slice_indices);
 private:
  static void ComputeCentroids(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches,
                               TPoint3D & out_centroid1,
                               TPoint3D & out_centroid2,
                               const std::vector<size_t> & slice_indices);
  static void ComputeRelativeCoordinates(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches,
                                         const TPoint3D & centroid1,
                                         const TPoint3D & centroid2,
                                         std::vector<std::pair<TPoint3D, TPoint3D>> & out_relative_coords,
                                         const std::vector<size_t> & slice_indices);
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
