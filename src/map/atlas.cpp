//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "atlas.h"
#include <settings.h>
#include <messages/messages.h>
#include <serialization/serialization_context.h>
#include <factories/camera_factory.h>

namespace orb_slam3 {
namespace map {
Atlas::Atlas() : current_map_(nullptr) {

}

Map * Atlas::GetCurrentMap() {
  if (nullptr == current_map_)
    CreateNewMap();

  return current_map_;
}

void Atlas::CreateNewMap() {
  current_map_ = new Map();
  if (Settings::Get().MessageRequested(messages::MAP_CREATED))
    messages::MessageProcessor::Instance().Enqueue(new messages::MapCreated(current_map_));
  maps_.insert(current_map_);
}

Atlas::~Atlas() {

}

void Atlas::SetCurrentMap(map::Map * map) {
  maps_.insert(map);
  current_map_ = map;
}

size_t Atlas::GetMapCount() const {
  return maps_.size();
}

const std::unordered_set<map::Map *> & Atlas::GetMaps() const {
  return maps_;
}

void Atlas::Serialize(std::ostream & ostream) const {

  std::unordered_set<const camera::ICamera *> cameras;
  std::unordered_set<const frame::SensorConstants *> sensor_constants;
  std::unordered_set<const features::IFeatureExtractor *> feature_extractors;

  for (const auto map: maps_) {
    for (auto kf: map->GetAllKeyFrames()) {
      cameras.insert(kf->GetCamera());
      sensor_constants.insert(kf->GetSensorConstants());
      feature_extractors.insert(kf->GetFeatureHandler()->GetFeatureExtractor());
    }
  }

  size_t camera_count = cameras.size();
  WRITE_TO_STREAM(camera_count, ostream);
  for (auto camera: cameras) {
    camera::CameraType cam_type = camera->Type();
    WRITE_TO_STREAM(cam_type, ostream);
    size_t cam_id = reinterpret_cast<size_t>(camera);
    WRITE_TO_STREAM(cam_id, ostream);
    camera->Serialize(ostream);
  }
  size_t sensor_constant_count = sensor_constants.size();
  WRITE_TO_STREAM(sensor_constant_count, ostream);
  for (auto sensor_constant: sensor_constants) {
    size_t sensor_constant_id = reinterpret_cast<size_t>(sensor_constant);
    WRITE_TO_STREAM(sensor_constant_id, ostream);
    sensor_constant->Serialize(ostream);
  }

  size_t feature_extractor_count = feature_extractors.size();
  WRITE_TO_STREAM(feature_extractor_count, ostream);
  for (auto feature_extractor: feature_extractors) {
    features::FeatureExtractorType fe_type = feature_extractor->Type();
    WRITE_TO_STREAM(fe_type, ostream);
    feature_extractor->Serialize(ostream);
  }

  size_t map_count = GetMapCount();
  WRITE_TO_STREAM(map_count, ostream);

  for (const auto map: maps_) {
    size_t map_id = reinterpret_cast<size_t>(map);
    WRITE_TO_STREAM(map_id, ostream);
    map->Serialize(ostream);
  }

}

void Atlas::Deserialize(std::istream & istream, serialization::SerializationContext & context) {
  size_t camera_count;
  READ_FROM_STREAM(camera_count, istream);
  for (size_t i = 0; i < camera_count; ++i) {
    camera::CameraType type;
    READ_FROM_STREAM(type, istream);
    size_t cam_id;
    READ_FROM_STREAM(cam_id, istream);

    auto camera = factories::CameraFactory::CreateCamera(type);
    camera->Deserialize(istream, context);
    context.cam_id[cam_id] = camera;
  }

  size_t sensor_constant_count;
  READ_FROM_STREAM(sensor_constant_count, istream);
  for (size_t i = 0; i < sensor_constant_count; ++i) {
    size_t sensor_constant_id;
    READ_FROM_STREAM(sensor_constant_id, istream);
    auto sensor_constant = new frame::SensorConstants;
    context.sc_id[sensor_constant_id] = sensor_constant;
    sensor_constant->Deserialize(istream, context);
  }

  size_t feature_extractor_count;
  READ_FROM_STREAM(feature_extractor_count, istream);
  for (size_t i = 0; i < feature_extractor_count; ++i) {
    features::FeatureExtractorType fe_type;
    READ_FROM_STREAM(fe_type, istream);
#warning implement factory and deserialize
  }

  size_t map_count;
  READ_FROM_STREAM(map_count, istream);
  for (size_t i = 0; i < map_count; ++i) {
    auto map = new map::Map();
    size_t map_id;
    READ_FROM_STREAM(map_id, istream);
    context.map_id[map_id] = map;
    map->Deserialize(istream, context);
    SetCurrentMap(map);
  }

}

}
}