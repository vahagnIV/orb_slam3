//
// Created by vahagn on 04/02/21.
//

#ifndef ORB_SLAM3_KANNALA_BRANDT_8_H
#define ORB_SLAM3_KANNALA_BRANDT_8_H

// == orb-slam3 ===
#include "idistortion_model.h"

namespace orb_slam3 {
namespace camera {

class KannalaBrandt8 : public IDistortionModel{
 public:
  static const int DistrortionSize = 8;
  typedef typename g2o::BaseVertex<CAMERA_PARAMS_COUNT + DistrortionSize, Eigen::Matrix<double, -1, 1>>::EstimateType
      EstimateType;
  // IDistortion
  KannalaBrandt8(EstimateType *estimate)
  : estimate_(estimate) {}
  bool DistortPoint(const HomogenousPoint & undistorted, HomogenousPoint & distorted) override;
  bool UnDistortPoint(const HomogenousPoint & distorted, HomogenousPoint & undistorted) override;
  void GetTransformationJacobian(const HomogenousPoint &point, JacobianType &out_jacobian) override ;
 public:
  typedef EstimateType::Scalar Scalar;

  inline const Scalar & K1() const noexcept { return (*estimate_)[4]; }
  inline const Scalar & K2() const noexcept { return (*estimate_)[5]; }
  inline const Scalar & P1() const noexcept { return (*estimate_)[6]; }
  inline const Scalar & P2() const noexcept { return (*estimate_)[7]; }
  inline const Scalar & K3() const noexcept { return (*estimate_)[8]; }
  inline const Scalar & K4() const noexcept { return (*estimate_)[9]; }
  inline const Scalar & K5() const noexcept { return (*estimate_)[10]; }
  inline const Scalar & K6() const noexcept { return (*estimate_)[11]; }

  void SetK1(Scalar k1) noexcept { (*estimate_)[4] = k1; }
  void SetK2(Scalar k2) noexcept { (*estimate_)[5] = k2; }
  void SetP1(Scalar p1) noexcept { (*estimate_)[6] = p1; }
  void SetP2(Scalar p2) noexcept { (*estimate_)[7] = p2; }
  void SetK3(Scalar k3) noexcept { (*estimate_)[8] = k3; }
  void SetK4(Scalar k4) noexcept { (*estimate_)[9] = k4; }
  void SetK5(Scalar k5) noexcept { (*estimate_)[10] = k5; }
  void SetK6(Scalar k6) noexcept { (*estimate_)[11] = k6; }
 protected:
  EstimateType *estimate_;


};

}
}

#endif //ORB_SLAM3_KANNALA_BRANDT_8_H
