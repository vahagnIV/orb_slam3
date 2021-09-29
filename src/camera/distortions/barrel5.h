//
// Created by vahagn on 04/02/21.
//

#ifndef ORB_SLAM3_KANNALA_BRANDT_5_H
#define ORB_SLAM3_KANNALA_BRANDT_5_H

// == orb-slam3 ===
#include "idistortion_model.h"

namespace orb_slam3 {
namespace camera {

class Barrel5 : public IDistortionModel {
 public:
  // IDistortion
  Barrel5()
      : k1_(0), k2_(0), p1_(0), p2_(0), k3_(0) {}
  bool DistortPoint(const HomogenousPoint &undistorted, HomogenousPoint &distorted) const override;
  bool UnDistortPoint(const HomogenousPoint &distorted, HomogenousPoint &undistorted) const override;
  void ComputeJacobian(const TPoint2D &point, JacobianType &out_jacobian) const override;
 public:

  inline const precision_t &K1() const noexcept { return k1_; }
  inline const precision_t &K2() const noexcept { return k2_; }
  inline const precision_t &P1() const noexcept { return p1_; }
  inline const precision_t &P2() const noexcept { return p2_; }
  inline const precision_t &K3() const noexcept { return k3_; }

  void SetK1(precision_t k1) noexcept { k1_ = k1; }
  void SetK2(precision_t k2) noexcept { k2_ = k2; }
  void SetP1(precision_t p1) noexcept { p1_ = p1; }
  void SetP2(precision_t p2) noexcept { p2_ = p2; }
  void SetK3(precision_t k3) noexcept { k3_ = k3; }
  void Serialize(std::ostream & ostream) const override;
  DistortionModelType Type() override;
  void Deserialize(std::istream &istream, serialization::SerializationContext &context) override;
 protected:
  precision_t k1_, k2_, p1_, p2_, k3_;

};

}
}

#endif //ORB_SLAM3_KANNALA_BRANDT_5_H
