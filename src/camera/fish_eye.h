//
// Created by vahagn on 04/02/21.
//

#ifndef ORB_SLAM3_FISH_EYE_H
#define ORB_SLAM3_FISH_EYE_H

// == orb-slam3 ==
#include "idistortion_model.h"

namespace orb_slam3 {
namespace camera {

class FishEye : public IDistortionModel {
 public:
  static const int DistrortionSize = 4;
  typedef typename g2o::BaseVertex<CAMERA_PARAMS_COUNT + DistrortionSize, Eigen::Matrix<double, -1, 1>>::EstimateType
      EstimateType;
  FishEye(EstimateType *estimate)
      : estimate_(estimate) {}
  // IDistortion

  bool DistortPoint(const HomogenousPoint &undistorted, HomogenousPoint &distorted) const override;
  bool UnDistortPoint(const HomogenousPoint &distorted, HomogenousPoint &undistorted) const override;
  void ComputeJacobian(const TPoint2D &point, JacobianType &out_jacobian) const override;

 public:
  typedef EstimateType::Scalar Scalar;

  inline const Scalar &K1() const noexcept { return (*estimate_)[4]; }
  inline const Scalar &K2() const noexcept { return (*estimate_)[5]; }
  inline const Scalar &K3() const noexcept { return (*estimate_)[6]; }
  inline const Scalar &K4() const noexcept { return (*estimate_)[7]; }

  void SetK1(Scalar k1) noexcept { (*estimate_)[4] = k1; }
  void SetK2(Scalar k2) noexcept { (*estimate_)[5] = k2; }
  void SetK3(Scalar k3) noexcept { (*estimate_)[6] = k3; }
  void SetK4(Scalar k4) noexcept { (*estimate_)[7] = k4; }
 protected:
  EstimateType *estimate_;

};

}
}

#endif //ORB_SLAM3_FISH_EYE_H