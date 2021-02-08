//
// Created by vahagn on 04/02/21.
//

#ifndef ORB_SLAM3_KANNALA_BRANDT_5_H
#define ORB_SLAM3_KANNALA_BRANDT_5_H
#include "idistortion_model.h"

namespace orb_slam3 {
namespace camera {

class KannalaBrandt5 : public IDistortionModel<5> {
 public:
  // IDistortion
  KannalaBrandt5(EstimateType * estimate);
  bool DistortPoint(const TPoint3D & undistorted, TPoint3D & distorted) override;
  bool UnDistortPoint(const TPoint3D & distorted, TPoint3D & undistorted) override;

 public:
  typedef std::decay<decltype(*estimate_)>::type::Scalar Scalar;

  inline const Scalar & K1() const noexcept { return (*estimate_)[4]; }
  inline const Scalar & K2() const noexcept { return (*estimate_)[5]; }
  inline const Scalar & P1() const noexcept { return (*estimate_)[6]; }
  inline const Scalar & P2() const noexcept { return (*estimate_)[7]; }
  inline const Scalar & K3() const noexcept { return (*estimate_)[8]; }

  void SetK1(Scalar k1) noexcept { (*estimate_)[4] = k1; }
  void SetK2(Scalar k2) noexcept { (*estimate_)[5] = k2; }
  void SetP1(Scalar p1) noexcept { (*estimate_)[6] = p1; }
  void SetP2(Scalar p2) noexcept { (*estimate_)[7] = p2; }
  void SetK3(Scalar k3) noexcept { (*estimate_)[8] = k3; }

};

}
}

#endif //ORB_SLAM3_KANNALA_BRANDT_5_H
