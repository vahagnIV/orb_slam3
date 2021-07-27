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
 public:
  // Setters
  void SetStagingPosition(const geometry::Pose & pose) {
    SetStagingPosition(pose.R, pose.T);
  }

  void SetStagingPosition(const g2o::SE3Quat & quaternion) {
    SetStagingPosition(quaternion.rotation().toRotationMatrix(), quaternion.translation());
  }

  void SetIdentity() {
    SetStagingPosition(TMatrix33::Identity(), TVector3D::Zero());
    ApplyStaging();
  }

 public:
  // Getters
  const geometry::Pose & GetPosition() const { return pose_; }
  const geometry::Pose & GetStagingPosition() const { return staging_pose_; }

  geometry::Pose GetPositionWithLock() const {
    std::unique_lock<std::mutex> lock(position_mutex_);
    return pose_;
  }

  void ApplyStaging() {
    pose_ = staging_pose_;
    inverse_pose_ = pose_.GetInversePose();
  }

  const geometry::Pose & GetInversePosition() const { return inverse_pose_; }

  geometry::Pose GetInversePositionWithLock() const {
    std::unique_lock<std::mutex> lock(position_mutex_);
    return inverse_pose_;
  }
 private:
  void SetStagingPosition(const TMatrix33 & R, const TPoint3D & T) {
    staging_pose_.R = R;
    staging_pose_.T = T;
  }

 private:
  geometry::Pose pose_;
  geometry::Pose inverse_pose_;
  geometry::Pose staging_pose_;

  mutable std::mutex position_mutex_;
};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_RIGID_OBJECT_H_
