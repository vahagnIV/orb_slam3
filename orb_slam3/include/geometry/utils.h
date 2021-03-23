//
// Created by vahagn on 22.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_GEOMETRY_UTILS_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_GEOMETRY_UTILS_H_

#include <typedefs.h>

namespace orb_slam3 {
namespace geometry {
namespace utils {

TMatrix33 SkewSymmetricMatrix(const TVector3D &vector);
void ComputeRelativeTransformation(const TMatrix33 &R_to,
                                   const TVector3D &T_to,
                                   const TMatrix33 &R_from,
                                   const TVector3D &T_from,
                                   TMatrix33 &out_R,
                                   TVector3D &out_T);

}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_GEOMETRY_UTILS_H_
