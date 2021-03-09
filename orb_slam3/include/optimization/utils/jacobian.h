//
// Created by vahagn on 09/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_UTILS_JACOBIAN_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_UTILS_JACOBIAN_H_

// === orb-slam3 ===
#include <typedefs.h>
#include <camera/idistortion_model.h>

namespace orb_slam3 {
namespace optimization {
namespace utils {

typedef Eigen::Matrix<precision_t, 2, 3> ProjectionJacobianType;

template<int DistrortionSize>
void ComputeJacobian(const TPoint3D &pt,
                     const camera::IDistortionModel<DistrortionSize> *distortion_model,
                     ProjectionJacobianType &out_jacobian);

}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_UTILS_JACOBIAN_H_
