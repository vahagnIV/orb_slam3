//
// Created by vahagn on 17/05/2021.
//

#ifndef ORB_SLAM3_SRC_OPTIMIZATION_MONOCULAR_OPTIMIZATION_H_
#define ORB_SLAM3_SRC_OPTIMIZATION_MONOCULAR_OPTIMIZATION_H_

#include <frame/monocular/monocular_frame.h>

namespace orb_slam3 {
namespace optimization {

void OptimizePose(frame::monocular::MonocularFrame * frame);

void OptimizeSim3(const frame::monocular::MonocularKeyFrame * const to_frame,
                  const frame::monocular::MonocularKeyFrame * const from_frame,
                  geometry::Sim3Transformation & in_out_transformation,
                  const std::unordered_map<map::MapPoint *, size_t> & matches,
                  const std::unordered_map<map::MapPoint *, int> & predicted_levels);



}
}
#endif //ORB_SLAM3_SRC_OPTIMIZATION_MONOCULAR_OPTIMIZATION_H_
