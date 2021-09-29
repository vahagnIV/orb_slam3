//
// Created by vahagn on 10.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_ICAMERA_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_ICAMERA_H_

#include "src/camera/distortions/idistortion_model.h"
#include "camera_type.h"

namespace orb_slam3 {

namespace serialization {
class SerializationContext;
}

namespace camera {

typedef Eigen::Matrix<precision_t, 2, 3> ProjectionJacobianType;

class ICamera {
 public:
  virtual CameraType Type() const = 0;
  virtual void ComputeJacobian(const TPoint3D & pt,
                               ProjectionJacobianType & out_jacobian) const = 0;
  virtual const IDistortionModel * GetDistortionModel() const = 0;
  virtual void Serialize(std::ostream & ostream) const = 0;
  virtual void Deserialize(std::istream & istream, serialization::SerializationContext & context) = 0;
  virtual ~ICamera() = default;

};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_ICAMERA_H_
