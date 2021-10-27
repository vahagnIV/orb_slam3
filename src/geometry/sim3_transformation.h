//
// Created by vahagn on 01/07/2021.
//

#ifndef ORB_SLAM3_SRC_GEOMETRY_SIM3_TRANSFORMATION_H_
#define ORB_SLAM3_SRC_GEOMETRY_SIM3_TRANSFORMATION_H_

#include "typedefs.h"
#include "pose.h"
namespace orb_slam3 {
namespace geometry {
struct Sim3Transformation {

  TVector3D Transform(const TPoint3D & point) const;

  Sim3Transformation GetInverse() const;

  Sim3Transformation operator*(const Sim3Transformation & other) const;
  Sim3Transformation operator*(const Pose & se3_other) const;
  friend Sim3Transformation operator*(const Pose & se3_other, const Sim3Transformation & self) ;

  friend std::ostream & operator<<(std::ostream & stream, const Sim3Transformation & p);

  void print(std::ostream & stream=std::cout) const;

  TMatrix33 R;
  TVector3D T;
  precision_t s = 1;
};
}
}
#endif //ORB_SLAM3_SRC_GEOMETRY_SIM3_TRANSFORMATION_H_
