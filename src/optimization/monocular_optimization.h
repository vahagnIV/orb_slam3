//
// Created by vahagn on 17/05/2021.
//

#ifndef ORB_SLAM3_SRC_OPTIMIZATION_MONOCULAR_OPTIMIZATION_H_
#define ORB_SLAM3_SRC_OPTIMIZATION_MONOCULAR_OPTIMIZATION_H_

#include <frame/monocular/monocular_frame.h>

namespace orb_slam3 {
namespace optimization {

void OptimizePose(frame::monocular::MonocularFrame * frame);

}
}
#endif //ORB_SLAM3_SRC_OPTIMIZATION_MONOCULAR_OPTIMIZATION_H_
