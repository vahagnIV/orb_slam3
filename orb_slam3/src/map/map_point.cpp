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

void MapPoint::AddObservation(const frame::FrameBase* frame, size_t feature_id) {
  obsevations_[frame] = feature_id;
}

void MapPoint::EraseObservation(const frame::FrameBase *frame) {
  obsevations_.erase(frame);
}

void MapPoint::Refresh() {
  ComputeDistinctiveDescriptor();
  UpdateNormalAndDepth();
}

void MapPoint::ComputeDistinctiveDescriptor() {

  std::vector<features::DescriptorType> descriptors;
  for (const auto & frame_id_pair: obsevations_) {
    frame_id_pair.first->AppendDescriptorsToList(frame_id_pair.second, descriptors);
  }
  const unsigned N = descriptors.size();
  int distances[N][N];
  for (size_t i = 0; i < N; ++i) {
    distances[i][i] = 0;
    for (size_t j = i + 1; j < N; ++j) {
      distances[i][j] = features::DescriptorDistance(descriptors[i], descriptors[j]);
      distances[j][i] = distances[i][j];
    }
  }

  int best_median = INT_MAX;
  int best_idx = 0;
  for (size_t i = 0; i < N; i++) {
    std::vector<int> dists(distances[i], distances[i] + N);
    sort(dists.begin(), dists.end());
    int median = dists[0.5 * (N - 1)];

    if (median < best_median) {
      best_median = median;
      best_idx = i;
    }
  }
  descriptor_ = descriptors[best_idx];

}

void MapPoint::UpdateNormalAndDepth() {
  std::vector<TVector3D> normals;
  normal_.setZero();
  for (const auto & frame_id_pair: obsevations_) {
    const auto & mp = frame_id_pair.first->MapPoint(frame_id_pair.second);
    auto normal = frame_id_pair.first->GetNormal(mp->estimate());
    normal_ += normal;
  }
  normal_.normalize();
}

}
}