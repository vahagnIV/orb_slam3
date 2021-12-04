//
// Created by vahagn on 29/09/2021.
//

#include "base_frame.h"
#include <serialization/serialization_context.h>
#include <factories/feature_handler_factory.h>

#include <utility>

namespace orb_slam3 {
namespace frame {

BaseFrame::BaseFrame(std::istream &istream, serialization::SerializationContext &context) : atlas_(context.atlas) {
  size_t time_created;
  READ_FROM_STREAM(time_created, istream);
  SetTimePoint(TimePoint::clock::from_time_t(time_created));

  size_t kf_id;
  READ_FROM_STREAM(kf_id, istream);
  SetId(kf_id);

  size_t filename_size;
  READ_FROM_STREAM(filename_size, istream);
  char * buffer = new char[filename_size];
  istream.read(buffer, filename_size);
  SetFilename(std::string(buffer, filename_size));
  delete[] buffer;

  size_t map_id;
  READ_FROM_STREAM(map_id, istream);
  SetMap(context.map_id[map_id]);

  size_t sensor_constant_id;
  READ_FROM_STREAM(sensor_constant_id, istream);
  SetSensorConstants(context.sc_id[sensor_constant_id]);

  geometry::Pose pose;
  istream >> pose;
  SetStagingPosition(pose);
  ApplyStaging();

  features::handlers::HandlerType handler_type;
  READ_FROM_STREAM(handler_type, istream);
  std::shared_ptr<features::handlers::BaseFeatureHandler>
      handler = factories::FeatureHandlerFactory::Create(handler_type, istream, context);

  SetFeatureHandler(handler);
}

void BaseFrame::SetMap(map::Map * map) {
    map_ = map;
}

BaseFrame::BaseFrame(TimePoint time_point,
                     std::string  filename,
                     const SensorConstants * sensor_constants,
                     size_t id,
                     map::Atlas * atlas) :
    time_point_(time_point),
    filename_(std::move(filename)),
    sensor_constants_(sensor_constants),
    id_(id),
    feature_handler_(nullptr),
    map_(nullptr),
    atlas_(atlas){}

void BaseFrame::SetTimePoint(TimePoint time_point) {
  time_point_ = time_point;
}

void BaseFrame::SetFilename(const std::string & filename) {
  filename_ = filename;
}

void BaseFrame::SetSensorConstants(const SensorConstants * constants) {
  sensor_constants_ = constants;
}

void BaseFrame::SetId(size_t id) {
  id_ = id;
}

void BaseFrame::SetFeatureHandler(const std::shared_ptr<const features::handlers::BaseFeatureHandler> & handler) {
  feature_handler_ = handler;
}

void BaseFrame::Serialize(std::ostream & stream) const {

  size_t time_created = GetTimeCreated().time_since_epoch().count();
  WRITE_TO_STREAM(time_created, stream);

  size_t kf_id = Id();
  WRITE_TO_STREAM(kf_id, stream);

  size_t filename_size = GetFilename().length();
  WRITE_TO_STREAM(filename_size, stream)
  stream.write(GetFilename().data(), GetFilename().length());

  size_t map_id = reinterpret_cast<size_t>(GetMap());
  assert(map_id > 0);
  WRITE_TO_STREAM(map_id, stream);

  size_t sensor_constant_id = reinterpret_cast<size_t>(GetSensorConstants());
  WRITE_TO_STREAM(sensor_constant_id, stream);

  stream << GetPosition();

  features::handlers::HandlerType handler_type = GetFeatureHandler()->Type();
  WRITE_TO_STREAM(handler_type, stream);
  GetFeatureHandler()->Serialize(stream);

  // In case the frame has additional info to write
  SerializeToStream(stream);

}

}
}