//
// Created by vahagn on 22/06/2021.
//

#ifndef ORB_SLAM3_SRC_GEOMETRY_SIM_3_SOLVER_H_
#define ORB_SLAM3_SRC_GEOMETRY_SIM_3_SOLVER_H_

#include "typedefs.h"
#include "pose.h"
namespace orb_slam3 {
namespace geometry {

class Sim3Solver {
 public:
  Sim3Solver(){};

  Pose ComputeSim3(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches) const;
 private:
  static void ComputeCentroids(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches,
                        TPoint3D & out_centroid1, TPoint3D & out_centroid2) ;
  static void ComputeRelativeCoordinates(const std::vector<std::pair<TPoint3D, TPoint3D>> & matches,
                                  const TPoint3D & centroid1,
                                  const TPoint3D & centroid2,
                                  std::vector<std::pair<TPoint3D, TPoint3D>> & out_relative_coords) ;
  static TMatrix33 ComputeM(std::vector<std::pair<TPoint3D, TPoint3D>> & relative_coords) ;

};

}
}
#endif //ORB_SLAM3_SRC_GEOMETRY_SIM_3_SOLVER_H_
