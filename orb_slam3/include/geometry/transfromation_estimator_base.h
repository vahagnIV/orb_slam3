//
// Created by vahagn on 08/02/21.
//

#ifndef ORB_SLAM3_TRANSFROMATION_ESTIMATOR_BASE_H
#define ORB_SLAM3_TRANSFROMATION_ESTIMATOR_BASE_H

// == orb-slam3 ===
#include <typedefs.h>

namespace orb_slam3 {
namespace geometry {

class TransfromationEstimatorBase {
 public:
  TransfromationEstimatorBase(precision_t sigma) :
      sigma_threshold_(sigma),
      sigma_threshold__square_(sigma*sigma),
      sigma_squared_inv_(1 / sigma / sigma) {}
 protected:
  const precision_t sigma_threshold_;
  const precision_t sigma_threshold__square_;
  const precision_t sigma_squared_inv_;

};
}
}

#endif //ORB_SLAM3_TRANSFROMATION_ESTIMATOR_BASE_H
