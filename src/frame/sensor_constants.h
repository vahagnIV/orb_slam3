//
// Created by vahagn on 13/05/2021.
//

#ifndef ORB_SLAM3_SRC_FRAME_SENSOR_CONSTANTS_H_
#define ORB_SLAM3_SRC_FRAME_SENSOR_CONSTANTS_H_

#include <typedefs.h>

namespace orb_slam3 {
namespace serialization {
class SerializationContext;
}
namespace frame {

struct SensorConstants {
  unsigned min_mp_disappearance_count;
  unsigned number_of_keyframe_to_search_lm;
  precision_t max_allowed_discrepancy;
  precision_t projection_search_radius_multiplier;
  precision_t projection_search_radius_multiplier_after_relocalization;
  precision_t projection_search_radius_multiplier_after_lost;
  unsigned min_number_of_edges_sim3_opt;
  precision_t sim3_optimization_threshold;
  precision_t sim3_optimization_huber_delta;
  void Serialize(std::ostream & ostream) const;
  void Deserialize(std::istream & istream, serialization::SerializationContext & context);

};

}
}

#endif //ORB_SLAM3_SRC_FRAME_SENSOR_CONSTANTS_H_
