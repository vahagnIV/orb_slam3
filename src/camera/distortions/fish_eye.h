//
// Created by vahagn on 04/02/21.
//

#ifndef ORB_SLAM3_FISH_EYE_H
#define ORB_SLAM3_FISH_EYE_H

// == orb-slam3 ==
#include "idistortion_model.h"

namespace orb_slam3 {
namespace serialization {
class SerializationContext;
}
namespace camera {

class FishEye : public IDistortionModel {
 public:

  FishEye();
  FishEye(std::istream &istream, serialization::SerializationContext &context);
  // IDistortion

  bool DistortPoint(const HomogenousPoint &undistorted, HomogenousPoint &distorted) const override;
  bool UnDistortPoint(const HomogenousPoint &distorted, HomogenousPoint &undistorted) const override;
  void ComputeJacobian(const TPoint2D &point, JacobianType &out_jacobian) const override;
  void ComputeGradientAndHessian(const HomogenousPoint &point,
                                 TVector3D &out_gradient,
                                 TMatrix33 &out_hessian) override;
 public:

  inline const precision_t &K1() const noexcept { return k1_; }
  inline const precision_t &K2() const noexcept { return k2_; }
  inline const precision_t &K3() const noexcept { return k3_; }
  inline const precision_t &K4() const noexcept { return k4_; }

  void SetK1(precision_t k1) noexcept { k1_ = k1; }
  void SetK2(precision_t k2) noexcept { k2_ = k2; }
  void SetK3(precision_t k3) noexcept { k3_ = k3; }
  void SetK4(precision_t k4) noexcept { k4_ = k4; }
  void Serialize(std::ostream & ostream) const override;
  DistortionModelType Type() override;
 protected:
  precision_t k1_, k2_, k3_, k4_;

};

}
}

#endif //ORB_SLAM3_FISH_EYE_H
