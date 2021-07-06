//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_BASE_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_BASE_FRAME_H_

#include <unordered_set>

#include <typedefs.h>
#include "frame_type.h"
#include <geometry/rigid_object.h>
#include "sensor_constants.h"
#include "map_point_visibility_params.h"
#include <features/handlers/base_feature_handler.h>

namespace orb_slam3 {

namespace map {
class MapPoint;
class Map;
}

namespace frame {

class BaseFrame : public geometry::RigidObject {
 public:
  friend std::ostream & operator<<(std::ostream & stream, const BaseFrame * frame) {
    frame->SerializeToStream(stream);
    stream << frame->GetPosition();
    stream << frame->GetInversePosition();
    size_t size = frame->filename_.length();
    stream.write((char *) &size, sizeof(size));
    stream.write(frame->filename_.data(), size);
    return stream;
  }

  BaseFrame(TimePoint time_point,
            const std::string & filename,
            const SensorConstants * sensor_constants,
            size_t id,
            const std::shared_ptr<const features::handlers::BaseFeatureHandler> & feature_handler = nullptr) :
      time_point_(time_point),
      filename_(filename),
      sensor_constants_(sensor_constants),
      id_(id),
      feature_handler_(feature_handler),
      map_(nullptr) {}
  ~BaseFrame() override = default;

 public:
  typedef std::unordered_set<map::MapPoint *> MapPointSet;

  virtual FrameType Type() const = 0;
  virtual void ListMapPoints(MapPointSet & out_map_points) const = 0;
  virtual const SensorConstants * GetSensorConstants() const { return sensor_constants_; }
  virtual precision_t GetSimilarityScore(const BaseFrame * other) const = 0;
  //virtual bool Relocalize(frame::KeyFrameDatabase * key_frame_database, orb_slam3::map::Atlas * atlas) = 0;
  size_t Id() const { return id_; }

  TimePoint GetTimeCreated() const { return time_point_; }
  const features::IFeatureExtractor * GetFeatureExtractor() const { return feature_handler_->GetFeatureExtractor(); }
  const std::string & GetFilename() const { return filename_; }
  const std::shared_ptr<const features::handlers::BaseFeatureHandler> & GetFeatureHandler() const { return feature_handler_; }

  void SetMap(map::Map * map) { map_ = map; }
  const map::Map * GetMap() const { return map_; }
  map::Map * GetMap() { return map_; }
 protected:
  virtual void SerializeToStream(std::ostream & stream) const = 0;

 protected:
  const TimePoint time_point_;
  const std::string filename_;
  const SensorConstants * sensor_constants_;
  size_t id_;
  std::shared_ptr<const features::handlers::BaseFeatureHandler> feature_handler_;
  map::Map * map_;
};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_BASE_FRAME_H_
