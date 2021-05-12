//
// Created by vahagn on 03/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_VISIBLE_MAP_POINT_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_VISIBLE_MAP_POINT_H_

#include "../map/map_point.h"

namespace orb_slam3 {
namespace frame {

struct VisibleMapPoint {
  map::MapPoint * map_point;
  precision_t window_size;
  TPoint2D position;
  int level;
};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_VISIBLE_MAP_POINT_H_
