//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_RIGID_OBJECT_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_RIGID_OBJECT_H_

// === stl ===
#include <mutex>

// === g2o ====
#include <g2o/types/slam3d/se3quat.h>

// === orb_slam3 ===
#include "pose.h"

namespace orb_slam3 {
namespace geometry {

class RigidObject {
 public:
  virtual ~RigidObject() = default;
  void SetPosition(const geometry::Pose & pose) {
    SetPosition(pose.R, pose.T);
  }

  void SetPosition(const g2o::SE3Quat & quaternion) {
    SetPosition(quaternion.rotation().toRotationMatrix(), quaternion.translation());
  }

  void SetIdentity() {
    TMatrix33 R;
    TPoint3D T;
    R.setIdentity();
    T.setZero();
    SetPosition(R, T);
  }

  const geometry::Pose & GetPosition() const { return pose_; }
  geometry::Pose GetPositionWithLock() const {
    std::unique_lock<std::mutex> lock(position_mutex_);
    return pose_;
  }
  const geometry::Pose & GetInversePosition() const { return inverse_pose_; }
  geometry::Pose GetIncersePositionWithLock() const {
    std::unique_lock<std::mutex> lock(position_mutex_);
    return inverse_pose_;
  }
 private:
  void SetPosition(const TMatrix33 & R, const TPoint3D & T) {
    pose_.R = R;
    pose_.T = T;
    inverse_pose_ = pose_.GetInversePose();
  }
  geometry::Pose pose_;
  geometry::Pose inverse_pose_;
  mutable std::mutex position_mutex_;
};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_RIGID_OBJECT_H_
