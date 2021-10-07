//
// Created by vahagn on 01/10/2021.
//
#include "sensor_constants.h"
#include <serialization/serialization_context.h>

namespace orb_slam3 {
namespace frame {

void SensorConstants::Serialize(ostream & ostream) const{
  WRITE_TO_STREAM(min_mp_disappearance_count, ostream);
  WRITE_TO_STREAM(number_of_keyframe_to_search_lm, ostream);
  WRITE_TO_STREAM(max_allowed_discrepancy, ostream);
  WRITE_TO_STREAM(projection_search_radius_multiplier, ostream);
  WRITE_TO_STREAM(projection_search_radius_multiplier_after_relocalization, ostream);
  WRITE_TO_STREAM(projection_search_radius_multiplier_after_lost, ostream);
  WRITE_TO_STREAM(min_number_of_edges_sim3_opt, ostream);
  WRITE_TO_STREAM(sim3_optimization_threshold, ostream);
  WRITE_TO_STREAM(sim3_optimization_huber_delta, ostream);
}

void SensorConstants::Deserialize(istream & istream, serialization::SerializationContext & context) {
  READ_FROM_STREAM(min_mp_disappearance_count, istream);
  READ_FROM_STREAM(number_of_keyframe_to_search_lm, istream);
  READ_FROM_STREAM(max_allowed_discrepancy, istream);
  READ_FROM_STREAM(projection_search_radius_multiplier, istream);
  READ_FROM_STREAM(projection_search_radius_multiplier_after_relocalization, istream);
  READ_FROM_STREAM(projection_search_radius_multiplier_after_lost, istream);
  READ_FROM_STREAM(min_number_of_edges_sim3_opt, istream);
  READ_FROM_STREAM(sim3_optimization_threshold, istream);
  READ_FROM_STREAM(sim3_optimization_huber_delta, istream);
}

}
}