//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "map_point.h"
#include <frame/key_frame.h>
#include <map/map.h>
#include <map/atlas.h>
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
    : position_changed_(false),
      observations_changed_(false),
      staging_desciptor_calculated_(false),
      staging_normal_calculated_(false),
      visible_(1),
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
  if (Settings::Get().MessageRequested(messages::MAP_POINT_CREATED))
    messages::MessageProcessor::Instance().Enqueue(new messages::MapPointCreated(this));
  ++counter_;
}

MapPoint::MapPoint(istream & istream, serialization::SerializationContext & context) {
  READ_FROM_STREAM(staging_max_invariance_distance_, istream);
  READ_FROM_STREAM(staging_min_invariance_distance_, istream);
  min_invariance_distance_ = staging_min_invariance_distance_;
  max_invariance_distance_ = staging_max_invariance_distance_;
  size_t map_id;
  READ_FROM_STREAM(map_id, istream);
  map_ = context.map_id[map_id];
  istream.read((char *) staging_position_.data(),
               staging_position_.size() * sizeof(decltype(staging_position_)::Scalar));
  istream.read((char *) normal_.data(), normal_.size() * sizeof(decltype(position_)::Scalar));
  READ_FROM_STREAM(first_observed_frame_id_, istream);
  READ_FROM_STREAM(visible_, istream);
  READ_FROM_STREAM(found_, istream);
  READ_FROM_STREAM(bad_flag_, istream);
  size_t descriptor_length;
  READ_FROM_STREAM(descriptor_length, istream);
  istream.read((char *) staging_descriptor_.data(), descriptor_length * sizeof(decltype(descriptor_)::Scalar));
  descriptor_ = staging_descriptor_;
  if (Settings::Get().MessageRequested(messages::MAP_CREATED))
    messages::MessageProcessor::Instance().Enqueue(new messages::MapPointCreated(this));
  ++counter_;
}

MapPoint::~MapPoint() {
  SetBad();
}

bool MapPoint::GetObservation(const frame::KeyFrame * key_frame, frame::Observation & out_observation) const {
  std::shared_lock<std::shared_mutex> lock(observation_mutex_);
  auto it = observations_.find(key_frame);
  if (it != observations_.end()) {
    out_observation = it->second;
    return true;
  }
  return false;
}

bool MapPoint::GetStagingObservation(const frame::KeyFrame * key_frame, frame::Observation & out_observation) const {
  auto it = staging_observations_.find(key_frame);
  if (it != staging_observations_.end()) {
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
  staging_observations_.emplace(observation.GetKeyFrame(), observation);
  observations_changed_ = true;
  staging_normal_calculated_ = false;
  staging_desciptor_calculated_ = false;
}

void MapPoint::EraseObservation(frame::KeyFrame * frame) {
  assert(staging_observations_.find(frame) != staging_observations_.end());
  staging_observations_.erase(frame);
  observations_changed_ = true;
  staging_normal_calculated_ = false;
  staging_desciptor_calculated_ = false;
}

void MapPoint::SetBad() {
  // TODO: Implement this
  bad_flag_ = true;
  {
    std::unique_lock<std::shared_mutex> lock(observation_mutex_);
    observations_.clear();
  }
  --counter_;
  if (Settings::Get().MessageRequested(messages::MAP_CREATED))
    messages::MessageProcessor::Instance().Enqueue(new messages::MapPointDeleted(this));
}

void MapPoint::SetMap(map::Map * map) {
  map_->EraseMapPoint(this);
  map->AddMapPoint(this);
  map_ = map;
}

void MapPoint::ComputeDistinctiveDescriptor() {

  if (staging_desciptor_calculated_)
    return;

  std::vector<features::DescriptorType> descriptors;
  for (const auto & observation: staging_observations_) {
    observation.second.AppendDescriptorsToList(descriptors);
  }
  const unsigned N = descriptors.size();
  if (N == 0) {
    return;
  }
  int distances[N][N];
  for (size_t i = 0; i < N; ++i) {
    distances[i][i] = 0;
    for (size_t j = i + 1; j < N; ++j) {
      distances[i][j] = GetMap()->GetAtlas()->GetFeatureExtractor()->ComputeDistance(descriptors[i], descriptors[j]);
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
  staging_descriptor_ = descriptors[best_idx];
  staging_desciptor_calculated_ = true;
}

void MapPoint::CalculateNormalStaging() {
  if (!position_changed_ && !observations_changed_)
    return;

  if (staging_normal_calculated_)
    return;
  staging_normal_.setZero();
  for (const auto & frame_id_pair: staging_observations_) {
    staging_normal_ += frame_id_pair.first->GetNormalFromStaging(staging_position_);
  }
  staging_normal_.normalize();
  staging_normal_calculated_ = true;
}

const TPoint3D MapPoint::GetPosition() const {
  std::shared_lock<std::shared_mutex> lock(position_mutex_);
  return position_;
}

const TVector3D MapPoint::GetNormal() const {
  std::shared_lock<std::shared_mutex> lock(observation_mutex_);
  return normal_;
}

const TPoint3D MapPoint::GetStagingPosition() const {
  return staging_position_;
}

const TVector3D MapPoint::GetStagingNormal() {

  CalculateNormalStaging();

  return staging_normal_;
}

void MapPoint::SetStagingPosition(const TPoint3D & position) {
  position_changed_ = true;
  staging_normal_calculated_ = false;
  staging_position_ = position;
}

void MapPoint::ApplyStaging() {

  bool raise_event = false;
  if (position_changed_ || observations_changed_) {
    raise_event = true;
    if (position_changed_) {
      std::unique_lock<std::shared_mutex> lock(position_mutex_);
      position_ = staging_position_;
    }

    CalculateNormalStaging();

    if (observations_changed_) {
      ComputeDistinctiveDescriptor();
      descriptor_ = staging_descriptor_;
    }
    {
      std::unique_lock<std::shared_mutex> lock(observation_mutex_);
      normal_ = staging_normal_;
      observations_ = staging_observations_;
      if (observations_changed_)
        descriptor_ = staging_descriptor_;
    }

    min_invariance_distance_ = staging_min_invariance_distance_;
    max_invariance_distance_ = staging_max_invariance_distance_;

    position_changed_ = false;
    observations_changed_ = false;
  }

  if (raise_event && Settings::Get().MessageRequested(messages::MessageType::MAP_POINT_GEOMETRY_UPDATED)) {
    messages::MessageProcessor::Instance().Enqueue(new messages::MapPointGeometryUpdated(this));
  }
}

MapPoint::MapType MapPoint::Observations() const { /// TODO change prototype
  std::shared_lock<std::shared_mutex> lock(position_mutex_);
  return observations_;
}

MapPoint::MapType MapPoint::StagingObservations() const {
  return staging_observations_;
}

size_t MapPoint::GetObservationCount() const {
  std::shared_lock<std::shared_mutex> lock(position_mutex_);
  return observations_.size();
}

size_t MapPoint::GetStagingObservationCount() const {
  return staging_observations_.size();
}

bool MapPoint::IsInKeyFrame(const frame::KeyFrame * keyframe) const {
  std::shared_lock<std::shared_mutex> lock(position_mutex_);
  return observations_.find(keyframe) != observations_.end();
}

const frame::Observation & MapPoint::Observation(const frame::KeyFrame * key_frame) const {
  std::shared_lock<std::shared_mutex> lock(position_mutex_);
  return observations_.find(key_frame)->second;
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
  WRITE_TO_STREAM(bad_flag_, ostream);
  size_t descriptor_length = descriptor_.size();
  WRITE_TO_STREAM(descriptor_length, ostream);
  ostream.write((char *) descriptor_.data(), descriptor_length * sizeof(decltype(descriptor_)::Scalar));
}

}
}
