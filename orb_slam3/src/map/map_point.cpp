//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "map/map_point.h"
#include "frame/frame_base.h"
#include "features/feature_utils.h"

namespace orb_slam3 {
namespace map {

MapPoint::MapPoint(const TPoint3D & point, precision_t max_invariance_distance, precision_t min_invariance_distance)
    : Identifiable(),
      position_(point),
      max_invariance_distance_(max_invariance_distance),
      min_invariance_distance_(min_invariance_distance) {
}

void MapPoint::AddObservation(frame::FrameBase *frame, size_t feature_id) {
  for (auto & obs : observations_) {
    obs.first->CovisibilityGraph().AddConnection(frame);
    frame->CovisibilityGraph().AddConnection(obs.first);
  }
  observations_[frame] = feature_id;
}

void MapPoint::EraseObservation(frame::FrameBase *frame) {
  for (auto & obs : observations_) {
    obs.first->CovisibilityGraph().RemoveConnection(frame);
    frame->CovisibilityGraph().RemoveConnection(obs.first);
  }
  observations_.erase((frame::FrameBase *) frame);
}

void MapPoint::Refresh() {
  ComputeDistinctiveDescriptor();
  UpdateNormalAndDepth();
}

void MapPoint::ComputeDistinctiveDescriptor() {

  std::vector<features::DescriptorType> descriptors;
  for (const auto & frame_id_pair: observations_) {
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
  normal_.setZero();
  for (const auto & frame_id_pair: observations_) {
    auto normal = frame_id_pair.first->GetNormal(position_);
    normal_ += normal;
  }
  normal_.normalize();
}

void MapPoint::SetPosition(const TPoint3D & position) {
  position_ = position;
}

g2o::VertexPointXYZ *MapPoint::CreateVertex() const {
  auto mp = new g2o::VertexPointXYZ();
  mp->setId(Id());
  mp->setMarginalized(true);
  mp->setEstimate(GetPosition());
  mp->setFixed(false);
  return mp;
}
}
}