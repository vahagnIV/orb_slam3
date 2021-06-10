//
// Created by vahagn on 15/02/21.
//

#ifndef ORB_SLAM3_POSE_H
#define ORB_SLAM3_POSE_H

// === optimization ===
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/sba/vertex_se3_expmap.h>

// == orb-slam3 ===
#include "../typedefs.h"

namespace orb_slam3 {
namespace geometry {

typedef g2o::SE3Quat Quaternion;
struct Pose {

  g2o::SE3Quat GetQuaternion() const {
    return Quaternion(R, T);
  }

  TVector3D Transform(const TPoint3D & point) const {
    return R * point + T;
  }

  Pose GetInversePose() const {
    return Pose{.R = R.transpose(), .T = -R.transpose() * T};
  }

  friend std::ostream & operator<<(std::ostream & stream, const Pose & p) {
    stream.write((char *)p.R.data(), p.R.size() * sizeof(decltype(p.R)::Scalar));
    stream.write((char *)p.T.data(), p.T.size() * sizeof(decltype(p.T)::Scalar));
//    stream << p.R << std::endl << p.T << std::endl;
    return stream;
  }

  void print() const {
    std::cout << R << std::endl << T << std::endl;
  }

  TMatrix33 R;
  TVector3D T;
};

}
}

#endif //ORB_SLAM3_POSE_H
