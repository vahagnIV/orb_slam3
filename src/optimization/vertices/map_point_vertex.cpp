//
// Created by vahagn on 12/05/2021.
//
#include "map_point_vertex.h"
#include <map/map_point.h>

namespace orb_slam3 {

namespace optimization {
namespace vertices {

MapPointVertex::MapPointVertex(map::MapPoint * map_point)  : map_point_(map_point) {
  setEstimate(map_point->GetStagingPosition());
  setMarginalized(true);
  setFixed(false);
}

}
}
}