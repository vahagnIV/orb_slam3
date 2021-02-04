//
// Created by vahagn on 04/02/21.
//

#ifndef ORB_SLAM3_FISH_EYE_H
#define ORB_SLAM3_FISH_EYE_H

#include "idistortion_model.h"

namespace orb_slam3 {
namespace camera {

class FishEye : public IDistortionModel<4>{
 public:
  // IDistortion
  FishEye(EstimateType * estimate);
  void DistortPoint(const TPoint2D & undistorted, TPoint2D & distorted) override;
  void UnDistortPoint(const TPoint2D & distorted, TPoint2D & undistorted) override;

 public:
  typedef std::decay<decltype(*estimate_)>::type::Scalar Scalar;

  inline const Scalar & K1() const noexcept { return (*estimate_)[4]; }
  inline const Scalar & K2() const noexcept { return (*estimate_)[5]; }
  inline const Scalar & K3() const noexcept { return (*estimate_)[6]; }
  inline const Scalar & K4() const noexcept { return (*estimate_)[7]; }


  void SetK1(Scalar k1) noexcept { (*estimate_)[4] = k1; }
  void SetK2(Scalar k2) noexcept { (*estimate_)[5] = k2; }
  void SetK3(Scalar k3) noexcept { (*estimate_)[6] = k3; }
  void SetK4(Scalar k4) noexcept { (*estimate_)[7] = k4; }
};

}
}

#endif //ORB_SLAM3_FISH_EYE_H
