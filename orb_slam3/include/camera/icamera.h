//
// Created by vahagn on 10.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_ICAMERA_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_ICAMERA_H_

#include <camera/idistortion_model.h>

namespace orb_slam3 {
namespace camera{

typedef Eigen::Matrix<precision_t, 2, 3> ProjectionJacobianType;

class ICamera {
 public:
  virtual void ComputeJacobian(const TPoint3D &pt,
                       ProjectionJacobianType &out_jacobian) const = 0;
  virtual const IDistortionModel * GetDistortionModel() const = 0;
  virtual ~ICamera() = default;
  virtual bool IsInFrustum(const HomogenousPoint & point) const = 0;

};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_ICAMERA_H_
