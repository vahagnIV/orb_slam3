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

  Pose();
  Pose(Pose &&other);
  Pose(const Pose &other);
  Pose(TMatrix33 R, TVector3D T);

  Pose &operator=(Pose &&other);
  Pose &operator=(const Pose &other);

  Pose operator*(const Pose &other) const;

  TVector3D Transform(const TPoint3D &point) const;

  Pose GetInversePose() const;

  friend std::ostream &operator<<(std::ostream &stream, const Pose &p);

  void print() const;

  TMatrix33 R;
  TVector3D T;
};

}
}

#endif //ORB_SLAM3_POSE_H
