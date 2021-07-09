//
// Created by vahagn on 01/07/2021.
//

#ifndef ORB_SLAM3_SRC_GEOMETRY_SIM3_TRANSFORMATION_H_
#define ORB_SLAM3_SRC_GEOMETRY_SIM3_TRANSFORMATION_H_

#include "typedefs.h"
namespace orb_slam3 {
namespace geometry {
struct Sim3Transformation {

  TVector3D Transform(const TPoint3D & point) const;

  Sim3Transformation GetInversePose() const;

  Sim3Transformation operator*(const Sim3Transformation & other) const;

  friend std::ostream & operator<<(std::ostream & stream, const Sim3Transformation & p);

  void print() const;

  TMatrix33 R;
  TVector3D T;
  precision_t s = 1;
};
}
}
#endif //ORB_SLAM3_SRC_GEOMETRY_SIM3_TRANSFORMATION_H_
