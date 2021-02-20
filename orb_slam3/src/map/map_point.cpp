//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include <map/map_point.h>

namespace orb_slam3 {
namespace map {

MapPoint::MapPoint(const TPoint3D & point) {
  this->setEstimate(point);
}

void MapPoint::AddObservation(const std::shared_ptr<frame::FrameBase> & frame, size_t feature_id) {
  obsevations_[frame] = feature_id;
}

}
}