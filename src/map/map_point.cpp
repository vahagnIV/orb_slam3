//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "map_point.h"
#include <frame/key_frame.h>

#include <utility>

namespace orb_slam3 {
namespace map {

std::atomic_uint64_t MapPoint::counter_(0);

MapPoint::MapPoint(TPoint3D point,
                   size_t first_observed_frame_id,
                   precision_t max_invariance_distance,
                   precision_t min_invariance_distance)
    : position_(std::move(point)),
      max_invariance_distance_(max_invariance_distance),
      min_invariance_distance_(min_invariance_distance),
      visible_(0),
      found_(0),
      bad_flag_(false),
      first_observed_frame_id_(first_observed_frame_id) {
  ++counter_;
}

MapPoint::~MapPoint() {
  --counter_;
}

void MapPoint::AddObservation(const frame::Observation & observation) {
  observations_.emplace(observation.GetKeyFrame(), observation);
//  observations_[observation.GetKeyFrame()] = observation;
}

void MapPoint::EraseObservation(frame::KeyFrame * frame) {
  std::unique_lock<std::mutex> lock(feature_mutex_);
  observations_.erase(frame);
  if(observations_.size() < 1)
    SetBad();
}

void MapPoint::SetBad() {
  // TODO: Implement this
  bad_flag_ = true;

}

void MapPoint::Refresh(const features::IFeatureExtractor * feature_extractor) {
  ComputeDistinctiveDescriptor(feature_extractor);
  UpdateNormalAndDepth();
}

void MapPoint::ComputeDistinctiveDescriptor(const features::IFeatureExtractor * feature_extractor) {

  std::vector<features::DescriptorType> descriptors;
  for (const auto & observation: observations_) {
    observation.second.AppendDescriptorsToList(descriptors);
  }
  const unsigned N = descriptors.size();
  int distances[N][N];
  for (size_t i = 0; i < N; ++i) {
    distances[i][i] = 0;
    for (size_t j = i + 1; j < N; ++j) {
      distances[i][j] = feature_extractor->ComputeDistance(descriptors[i], descriptors[j]);
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

const MapPoint::MapType MapPoint::Observations() const {
  std::unique_lock<std::mutex> lock(feature_mutex_);
  return observations_;
}

size_t MapPoint::GetObservationCount() const {
  std::unique_lock<std::mutex> lock(feature_mutex_);
  return observations_.size();
}

bool MapPoint::IsInKeyFrame(frame::KeyFrame * keyframe) {
  std::unique_lock<std::mutex> lock(feature_mutex_);
  return observations_.find(keyframe) != observations_.end();
}

}
}