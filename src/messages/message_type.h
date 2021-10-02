//
// Created by vahagn on 13/08/2021.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_MESSAGE_TYPE_H_
#define ORB_SLAM3_SRC_MESSAGES_MESSAGE_TYPE_H_
#include <string>
namespace orb_slam3 {
namespace messages {

enum MessageType {
  MAP_CREATED = 1,
  TRACKING_INFO = 1 << 1,
  /*reserved = 4,*/
  KEYFRAME_CREATED = 1 << 3,
  MAP_POINT_CREATED = 1 << 4,
  KEYFRAME_DELETED = 1 << 5,
  MAP_POINT_DELETED = 1 << 6,
  OBSERVATION_ADDED = 1 << 7,
  OBSERVATION_DELETED = 1 << 8,
  KEYFRAME_POSITION_UPDATED = 1 << 9,
  KEYFRAME_COVISIBILITY_UPDATED = 1 << 10,
  MAP_POINT_GEOMETRY_UPDATED = 1 << 11
};


std::string to_string(MessageType type);

}
}
#endif //ORB_SLAM3_SRC_MESSAGES_MESSAGE_TYPE_H_
