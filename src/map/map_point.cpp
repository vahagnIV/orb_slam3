//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "map_point.h"
#include <frame/key_frame.h>
#include <map/map.h>
#include <settings.h>
#include <messages/messages.h>
#include <utility>
#include <serialization/serialization_context.h>

namespace orb_slam3 {
namespace map {

std::atomic_uint64_t MapPoint::counter_(0);

MapPoint::MapPoint(TPoint3D point,
                   size_t first_observed_frame_id,
                   precision_t max_invariance_distance,
                   precision_t min_invariance_distance,
                   Map * map)
    : visible_(1),
      found_(1),
      map_(map),
      bad_flag_(false),
      first_observed_frame_id_(first_observed_frame_id),
      replaced_map_point_(nullptr) {
  SetStagingPosition(point);
  SetStagingMinInvarianceDistance(min_invariance_distance);
  SetStagingMaxInvarianceDistance(max_invariance_distance);
  ApplyStaging();
  map_->AddMapPoint(this);
  if (Settings::Get().MessageRequested(messages::MAP_CREATED))
    messages::MessageProcessor::Instance().Enqueue(new messages::MapPointCreated(this));
  ++counter_;
}

MapPoint::MapPoint() : bad_flag_(false) {
// TODO: Initialize fields
}

MapPoint::~MapPoint() {
  --counter_;
}

bool MapPoint::GetObservation(const frame::KeyFrame * key_frame, frame::Observation & out_observation) const {
  auto it = observations_.find(key_frame);
  if (it != observations_.end()) {
    out_observation = it->second;
    return true;
  }
  return false;
}

void MapPoint::SetReplaced(map::MapPoint * replaced) {
  SetBad();
  replaced_map_point_ = replaced;
}

map::MapPoint * MapPoint::GetReplaced() {
  return replaced_map_point_;
}

void MapPoint::AddObservation(const frame::Observation & observation) {
  assert(!IsBad());
  observations_.emplace(observation.GetKeyFrame(), observation);
}

void MapPoint::EraseObservation(frame::KeyFrame * frame) {
  //std::unique_lock<std::mutex> lock(feature_mutex_); TODO
  auto it = observations_.find(frame);
  assert(it != observations_.end());
  observations_.erase(it);
}

void MapPoint::SetBad() {
  // TODO: Implement this
  bad_flag_ = true;
  observations_.clear();
  if (Settings::Get().MessageRequested(messages::MAP_CREATED))
    messages::MessageProcessor::Instance().Enqueue(new messages::MapPointDeleted(this));
}

void MapPoint::SetMap(map::Map * map) {
  map_->EraseMapPoint(this);
  map_ = map;
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

void MapPoint::CalculateNormalStaging() {
  staging_normal_.setZero();
  for (const auto & frame_id_pair: observations_) {
    auto normal = frame_id_pair.first->GetNormalFromStaging(staging_position_);
    staging_normal_ += normal;
  }
  staging_normal_.normalize();
}

void MapPoint::SetStagingPosition(const TPoint3D & position) {
  staging_position_ = position;
}

void MapPoint::ApplyStagingPosition() {
  std::unique_lock<std::recursive_mutex> lock(position_mutex_);
  position_ = staging_position_;
}

void MapPoint::ApplyNormalStaging() {
  std::unique_lock<std::recursive_mutex> lock(normal_mutex_);
  normal_ = staging_normal_;
}

const TPoint3D & MapPoint::GetPositionWithLock() const {
  std::unique_lock<std::recursive_mutex> lock(position_mutex_);
  return position_;
}
const TVector3D & MapPoint::GetNormalWithLock() const {
  std::unique_lock<std::recursive_mutex> lock(normal_mutex_);
  return normal_;
}

void MapPoint::ApplyMinMaxInvDistanceStaging() {
  min_invariance_distance_ = staging_min_invariance_distance_;
  max_invariance_distance_ = staging_max_invariance_distance_;
}

void MapPoint::ApplyStaging() {
  ApplyStagingPosition();
  ApplyNormalStaging();
  ApplyMinMaxInvDistanceStaging();
}

const MapPoint::MapType MapPoint::Observations() const { /// TODO change prototype
  return observations_;
}

size_t MapPoint::GetObservationCount() const {
//  std::unique_lock<std::mutex> lock(feature_mutex_);
  return observations_.size();
}

const TPoint3D & MapPoint::GetPosition() const {
  std::unique_lock<std::recursive_mutex> lock(position_mutex_);
  return position_;
}

bool MapPoint::IsInKeyFrame(const frame::KeyFrame * keyframe) const {
//  std::unique_lock<std::mutex> lock(feature_mutex_);
  return observations_.find(keyframe) != observations_.end();
}

const frame::Observation & MapPoint::Observation(const frame::KeyFrame * key_frame) const {
  return observations_.find(key_frame)->second;
}

void MapPoint::LockObservationsContainer() const {
  observation_mutex_.lock();
}

void MapPoint::UnlockObservationsContainer() const {
  observation_mutex_.unlock();
}

void MapPoint::Serialize(std::ostream & ostream) const {

  WRITE_TO_STREAM(max_invariance_distance_, ostream);
  WRITE_TO_STREAM(min_invariance_distance_, ostream);
  size_t map_id = reinterpret_cast<size_t>(map_);
  WRITE_TO_STREAM(map_id, ostream);
  ostream.write((char *) position_.data(), position_.size() * sizeof(decltype(position_)::Scalar));
  ostream.write((char *) normal_.data(), normal_.size() * sizeof(decltype(position_)::Scalar));
  WRITE_TO_STREAM(first_observed_frame_id_, ostream);
  WRITE_TO_STREAM(visible_, ostream);
  WRITE_TO_STREAM(found_, ostream);
  size_t descriptor_length = descriptor_.size();
  WRITE_TO_STREAM(descriptor_length, ostream);
  ostream.write((char *) descriptor_.data(), descriptor_length * sizeof(decltype(descriptor_)::Scalar));
}

void MapPoint::Deserialize(std::istream & istream, serialization::SerializationContext & context) {
//  istream.rdbuf()
  READ_FROM_STREAM(staging_max_invariance_distance_, istream);
  READ_FROM_STREAM(staging_min_invariance_distance_, istream);
  ApplyMinMaxInvDistanceStaging();
  size_t map_id;
  READ_FROM_STREAM(map_id, istream);
  map_ = context.map_id[map_id];
  istream.read((char *) staging_position_.data(),
               staging_position_.size() * sizeof(decltype(staging_position_)::Scalar));
  istream.read((char *) normal_.data(), normal_.size() * sizeof(decltype(position_)::Scalar));
  READ_FROM_STREAM(first_observed_frame_id_, istream);
  READ_FROM_STREAM(visible_, istream);
  READ_FROM_STREAM(found_, istream);
  size_t descriptor_length;
  READ_FROM_STREAM(descriptor_length, istream);
  istream.read((char *) descriptor_.data(), descriptor_length * sizeof(decltype(descriptor_)::Scalar));
  ApplyStaging();

}

}
}
