//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "map/map_point.h"
#include "frame/frame_base.h"
#include "features/feature_utils.h"

namespace orb_slam3 {
namespace map {

MapPoint::MapPoint(const TPoint3D & point) {
  this->setEstimate(point);
}

void MapPoint::AddObservation(const std::shared_ptr<frame::FrameBase> & frame, size_t feature_id) {
  obsevations_[frame] = feature_id;
}

void MapPoint::Refresh() {
  ComputeDistinctiveDescriptor();
  UpdateNormalAndDepth();
}

void MapPoint::ComputeDistinctiveDescriptor() {
//  std::map<frame::FrameBase * , int>
  std::vector<features::DescriptorType> descriptors;
  for (const auto & frame_id_pair: obsevations_) {
    frame_id_pair.first->AppendDescriptorsToList(frame_id_pair.second, descriptors);
  }
  const int N = descriptors.size();
  int distances[N][N];
  for (int i = 0; i < N; ++i) {
    distances[i][i] = 0;
    for (int j = i + 1; j < N; ++j) {
      distances[i][j] = features::DescriptorDistance(descriptors[i], descriptors[j]);
      distances[j][i] = distances[i][j];
    }
  }

}

void MapPoint::UpdateNormalAndDepth() {

}

}
}