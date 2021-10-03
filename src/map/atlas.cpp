//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "atlas.h"
#include <settings.h>
#include <messages/messages.h>
#include <serialization/serialization_context.h>
#include <factories/camera_factory.h>
#include <factories/feature_extractor_factory.h>
#include <factories/feature_handler_factory.h>
#include <frame/database/ikey_frame_database.h>

namespace orb_slam3 {
namespace map {

Atlas::Atlas(features::IFeatureExtractor *feature_extractor, frame::IKeyFrameDatabase *keyframe_database)
    : current_map_(nullptr),
      maps_(),
      feature_extractor_(feature_extractor),
      key_frame_database_(keyframe_database) {

}

Atlas::Atlas(std::istream &istream, serialization::SerializationContext &context) : current_map_(nullptr) {
  context.atlas = this;

  frame::KeyframeDatabaseType kf_db_type;
  READ_FROM_STREAM(kf_db_type, istream);
  key_frame_database_ = factories::FeatureHandlerFactory::CreateKeyFrameDatabase(kf_db_type, istream, context);

  features::FeatureExtractorType fe_type;
  READ_FROM_STREAM(fe_type, istream);
  feature_extractor_ = factories::FeatureExtractorFactory::Create(fe_type, istream, context);

  size_t camera_count;
  READ_FROM_STREAM(camera_count, istream);
  for (size_t i = 0; i < camera_count; ++i) {
    camera::CameraType type;
    READ_FROM_STREAM(type, istream);
    size_t cam_id;
    READ_FROM_STREAM(cam_id, istream);

    auto camera = factories::CameraFactory::CreateCamera(type, istream, context);
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

  size_t map_count;
  READ_FROM_STREAM(map_count, istream);
  for (size_t i = 0; i < map_count; ++i) {
    auto map = new map::Map(this);
    size_t map_id;
    READ_FROM_STREAM(map_id, istream);
    context.map_id[map_id] = map;
    map->Deserialize(istream, context);
    SetCurrentMap(map);
  }
}

Map *Atlas::GetCurrentMap() {
  if (nullptr == current_map_)
    CreateNewMap();

  return current_map_;
}

void Atlas::CreateNewMap() {
  current_map_ = new Map(this);
  if (Settings::Get().MessageRequested(messages::MAP_CREATED))
    messages::MessageProcessor::Instance().Enqueue(new messages::MapCreated(current_map_));
  maps_.insert(current_map_);
}

Atlas::~Atlas() {
  delete feature_extractor_;
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

  frame::KeyframeDatabaseType kf_db_type = key_frame_database_->Type();
  WRITE_TO_STREAM(kf_db_type, ostream);
  key_frame_database_->Serialize(ostream);

  features::FeatureExtractorType fe_type = feature_extractor_->Type();
  WRITE_TO_STREAM(fe_type, ostream);
  feature_extractor_->Serialize(ostream);

  std::unordered_set<const camera::ICamera *> cameras;
  std::unordered_set<const frame::SensorConstants *> sensor_constants;

  for (const auto map: maps_) {
    for (auto kf: map->GetAllKeyFrames()) {
      cameras.insert(kf->GetCamera());
      sensor_constants.insert(kf->GetSensorConstants());
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

  size_t map_count = GetMapCount();
  WRITE_TO_STREAM(map_count, ostream);

  for (const auto map: maps_) {
    size_t map_id = reinterpret_cast<size_t>(map);
    WRITE_TO_STREAM(map_id, ostream);
    map->Serialize(ostream);
  }
}

const features::IFeatureExtractor *Atlas::GetFeatureExtractor() const {
  return feature_extractor_;
}

frame::IKeyFrameDatabase *Atlas::GetKeyframeDatabase() const {
  return key_frame_database_;
}

}
}