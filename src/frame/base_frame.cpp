//
// Created by vahagn on 29/09/2021.
//

#include "base_frame.h"
#include <serialization/serialization_context.h>
#include <factories/feature_handler_factory.h>

namespace orb_slam3 {
namespace frame {

BaseFrame::BaseFrame()
    : time_point_(), filename_(), sensor_constants_(nullptr), id_(0), feature_handler_(nullptr), map_(
    nullptr) {
}

BaseFrame::BaseFrame(TimePoint time_point,
                     const std::string & filename,
                     const SensorConstants * sensor_constants,
                     size_t id,
                     const std::shared_ptr<const features::handlers::BaseFeatureHandler> & feature_handler) :
    time_point_(time_point),
    filename_(filename),
    sensor_constants_(sensor_constants),
    id_(id),
    feature_handler_(feature_handler),
    map_(nullptr) {}

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

  size_t camera_id = reinterpret_cast<size_t>(GetCamera());
  WRITE_TO_STREAM(camera_id, stream);

  stream << GetPosition();

  features::handlers::HandlerType handler_type = GetFeatureHandler()->Type();
  WRITE_TO_STREAM(handler_type, stream);
  GetFeatureHandler()->Serialize(stream);

  // In case the frame has additional info to write
  SerializeToStream(stream);

}

void BaseFrame::Deserialize(std::istream & stream, serialization::SerializationContext & context) {
  size_t time_created;
  READ_FROM_STREAM(time_created, stream);
  SetTimePoint(TimePoint::clock::from_time_t(time_created));

  size_t kf_id;
  READ_FROM_STREAM(kf_id, stream);
  SetId(kf_id);

  size_t filename_size;
  READ_FROM_STREAM(filename_size, stream);
  char * buffer = new char[filename_size];
  stream.read(buffer, filename_size);
  SetFilename(std::string(buffer, filename_size));

  size_t map_id;
  READ_FROM_STREAM(map_id, stream);
  SetMap(context.map_id[map_id]);

  size_t camera_id;
  READ_FROM_STREAM(camera_id, stream);
  SetCamera(context.cam_id[camera_id]);

  geometry::Pose pose;
  stream >> pose;
  SetStagingPosition(pose);
  ApplyStaging();

  features::handlers::HandlerType handler_type;
  READ_FROM_STREAM(handler_type, stream);
  std::shared_ptr<features::handlers::BaseFeatureHandler>
      handler = factories::FeatureHandlerFactory::Create(handler_type, context);
  handler->Deserialize(stream, context);
  SetFeatureHandler(handler);

  // In case the frame has additional info to write
  DeSerializeFromStream(stream, context);

}

}
}