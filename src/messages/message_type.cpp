//
// Created by vahagn on 13/08/2021.
//

#include "message_type.h"

namespace orb_slam3 {
namespace messages {

std::string to_string(MessageType type) {
  switch (type) {
    case MAP_CREATED:return "Map created";
    case TRACKING_INFO:return "Tracking info";
    case KEYFRAME_CREATED:return "Keyframe created";
    case KEYFRAME_DELETED:return "Keyframe deleted";
    case MAP_POINT_CREATED:return "Map Point created";
    case MAP_POINT_DELETED:return "Map Point deleted";
    case OBSERVATION_ADDED:return "Observation added";
    default:return "Unknown";
  }
}

}
}