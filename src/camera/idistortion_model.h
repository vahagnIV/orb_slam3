//
// Created by vahagn on 04/02/21.
//

#ifndef ORB_SLAM3_I_DISTORTION_MODEL_H
#define ORB_SLAM3_I_DISTORTION_MODEL_H

// === optimization ===
#include <g2o/core/base_vertex.h>

// == orb-slam3 ===
#include "../typedefs.h"

namespace orb_slam3 {
namespace camera {


class IDistortionModel {
 public:

  typedef Eigen::Matrix<double, 2, 2> JacobianType;

  virtual bool DistortPoint(const HomogenousPoint &undistorted,
                            HomogenousPoint &distorted) const = 0;
  virtual bool UnDistortPoint(const HomogenousPoint &distorted,
                              HomogenousPoint &undistorted) const = 0;
  virtual void ComputeJacobian(const TPoint2D &point, JacobianType &out_jacobian) const = 0;
  virtual ~IDistortionModel() = default;

};

}
}
#endif //ORB_SLAM3_I_DISTORTION_MODEL_H