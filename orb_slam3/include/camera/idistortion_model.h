//
// Created by vahagn on 04/02/21.
//

#ifndef ORB_SLAM3_I_DISTORTION_MODEL_H
#define ORB_SLAM3_I_DISTORTION_MODEL_H

// === optimization ===
#include <g2o/core/base_vertex.h>

// == orb-slam3 ===
#include <typedefs.h>

namespace orb_slam3 {
namespace camera {

template<int DistrortionSize>
class IDistortionModel {
 public:
  typedef typename g2o::BaseVertex<4 + DistrortionSize, Eigen::Matrix<double, -1, 1>>::EstimateType EstimateType;
  IDistortionModel(EstimateType * estimate)
      : estimate_(estimate) {}


  virtual bool DistortPoint(const HomogenousPoint & undistorted,
                            HomogenousPoint & distorted) = 0;
  virtual bool UnDistortPoint(const HomogenousPoint & distorted,
                              HomogenousPoint & undistorted) = 0;
  virtual ~IDistortionModel() = default;
 protected:
   EstimateType * estimate_;
};

}
}
#endif //ORB_SLAM3_I_DISTORTION_MODEL_H
