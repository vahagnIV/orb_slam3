//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_BASE_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_BASE_FRAME_H_

#include <unordered_set>

#include <typedefs.h>
#include "frame_type.h"
#include <features/ifeature_extractor.h>
#include <geometry/rigid_object.h>
#include "sensor_constants.h"
#include "visible_map_point.h"

namespace orb_slam3 {

namespace map {
class MapPoint;
}

namespace frame {

class BaseFrame : public geometry::RigidObject {
 public:
  friend std::ostream & operator<<(std::ostream & stream, const BaseFrame * frame){
    frame->SerializeToStream(stream);
    stream << frame->GetPosition();
    stream << frame->GetInversePosition();
    size_t size = frame->filename_.length();
    stream.write((char *)&size, sizeof(size));
    stream.write(frame->filename_.data(), size);
    return stream;
  }

  BaseFrame(TimePoint time_point,
            const std::string & filename,
            const features::IFeatureExtractor * feature_extractor,
            const features::BowVocabulary * vocabulary,
            const SensorConstants * sensor_constants,
            size_t id) :
      time_point_(time_point),
      filename_(filename),
      feature_extractor_(feature_extractor),
      vocabulary_(vocabulary),
      sensor_constants_(sensor_constants),
      id_(id) {}
  ~BaseFrame() override = default;

 public:
  typedef std::unordered_set<map::MapPoint *> MapPointSet;

  virtual void ComputeBow() = 0;
  virtual FrameType Type() const = 0;
  virtual void ListMapPoints(MapPointSet & out_map_points) const = 0;
  virtual const SensorConstants * GetSensorConstants() const { return sensor_constants_; }
  size_t Id() const { return id_; }

  TimePoint GetTimeCreated() const { return time_point_; }
  const features::IFeatureExtractor * GetFeatureExtractor() const { return feature_extractor_; }
  const std::string & GetFilename() const { return filename_; }
  const features::BowVocabulary * GetVocabulary() const { return vocabulary_; }
 protected:
  virtual void SerializeToStream(std::ostream & stream) const = 0;

 protected:
  const TimePoint time_point_;
  const std::string filename_;
  const features::IFeatureExtractor * feature_extractor_;
  const features::BowVocabulary * vocabulary_;
  const SensorConstants * sensor_constants_;
  size_t id_;
};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_BASE_FRAME_H_
