//
// Created by vahagn on 13/05/2021.
//

#ifndef ORB_SLAM3_SRC_FRAME_SENSOR_CONSTANTS_H_
#define ORB_SLAM3_SRC_FRAME_SENSOR_CONSTANTS_H_
namespace orb_slam3 {
namespace frame {

struct SensorConstants {
  unsigned max_mp_disappearance_count;
  unsigned number_of_keyframe_to_search_lm;

};

}
}

#endif //ORB_SLAM3_SRC_FRAME_SENSOR_CONSTANTS_H_
