//
// Created by vahagn on 04/02/21.
//

#ifndef ORB_SLAM3_KANNALA_BRANDT_8_H
#define ORB_SLAM3_KANNALA_BRANDT_8_H

// == orb-slam3 ===
#include "idistortion_model.h"

namespace orb_slam3 {
namespace serialization {
class SerializationContext;
}
namespace camera {

class Barrel8 : public IDistortionModel {
 public:
  // IDistortion
  Barrel8();
  Barrel8(std::istream &istream, serialization::SerializationContext &context);
  bool DistortPoint(const HomogenousPoint &undistorted, HomogenousPoint &distorted) const override;
  bool UnDistortPoint(const HomogenousPoint &distorted, HomogenousPoint &undistorted) const override;
  void ComputeJacobian(const TPoint2D &point, JacobianType &out_jacobian) const override;
 public:

  inline const precision_t &K1() const noexcept { return k1_; }
  inline const precision_t &K2() const noexcept { return k2_; }
  inline const precision_t &P1() const noexcept { return p1_; }
  inline const precision_t &P2() const noexcept { return p2_; }
  inline const precision_t &K3() const noexcept { return k3_; }
  inline const precision_t &K4() const noexcept { return k4_; }
  inline const precision_t &K5() const noexcept { return k5_; }
  inline const precision_t &K6() const noexcept { return k6_; }

  void SetK1(precision_t k1) noexcept { k1_ = k1; }
  void SetK2(precision_t k2) noexcept { k2_ = k2; }
  void SetP1(precision_t p1) noexcept { p1_ = p1; }
  void SetP2(precision_t p2) noexcept { p2_ = p2; }
  void SetK3(precision_t k3) noexcept { k3_ = k3; }
  void SetK4(precision_t k4) noexcept { k4_ = k4; }
  void SetK5(precision_t k5) noexcept { k5_ = k5; }
  void SetK6(precision_t k6) noexcept { k6_ = k6; }
  void Serialize(std::ostream & ostream) const override;
  DistortionModelType Type() override;
 protected:
  precision_t k1_, k2_, k3_, k4_, k5_, k6_;
  precision_t p1_, p2_;

};

}
}

#endif //ORB_SLAM3_KANNALA_BRANDT_8_H
