//
// Created by vahagn on 15/02/21.
//

#ifndef ORB_SLAM3_POSE_H
#define ORB_SLAM3_POSE_H

// === g2o ===
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/sba/vertex_se3_expmap.h>

// == orb-slam3 ===
#include <typedefs.h>

namespace orb_slam3 {
namespace geometry {

typedef g2o::VertexSE3Expmap Pose;
typedef g2o::SE3Quat Quaternion;

}
}

#endif //ORB_SLAM3_POSE_H
