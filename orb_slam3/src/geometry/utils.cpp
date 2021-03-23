//
// Created by vahagn on 22.03.21.
//

#include "geometry/utils.h"

namespace orb_slam3 {
namespace geometry {
namespace utils {

TMatrix33 SkewSymmetricMatrix(const TVector3D &vector) {
  TMatrix33 result;
  result << 0, -vector[2], vector[1],
      vector[2], 0, -vector[0],
      -vector[1], vector[0], 0;
  return result;
}

void ComputeRelativeTransformation(const TMatrix33 &R_to,
                                   const TVector3D &T_to,
                                   const TMatrix33 &R_from,
                                   const TVector3D &T_from,
                                   TMatrix33 &out_R,
                                   TVector3D &out_T) {
  out_R = R_to * R_from.transpose();
  out_T = -out_R * T_from + T_to;
}

}
}
}