//
// Created by vahagn on 15/02/21.
//

#ifndef ORB_SLAM3_POSE_H
#define ORB_SLAM3_POSE_H

// == orb-slam3 ===
#include <typedefs.h>

namespace orb_slam3 {
namespace geometry {

struct Pose {
  TMatrix33 R;
  TVector3D T;
};

}
}

#endif //ORB_SLAM3_POSE_H
