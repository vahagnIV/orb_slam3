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

}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_GEOMETRY_UTILS_H_
