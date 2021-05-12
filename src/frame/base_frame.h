//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_BASE_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_BASE_FRAME_H_

#include <unordered_set>

#include "frame_type.h"
#include <geometry/rigid_object.h>
#include <features/ifeature_extractor.h>

namespace orb_slam3 {
namespace map {
class MapPoint;
}
namespace frame {

class BaseFrame : public geometry::RigidObject {
 public:
  BaseFrame(TimePoint time_point,
            const std::string & filename,
            const features::IFeatureExtractor * feature_extractor,
            const features::BowVocabulary * vocabulary,
            size_t id) :
      time_point_(time_point),
      filename_(filename),
      feature_extractor_(feature_extractor),
      vocabulary_(vocabulary),
      id_(id){}
  virtual ~BaseFrame() = default;

 public:
  typedef std::unordered_set<map::MapPoint *> MapPointSet;

  virtual FrameType Type() const = 0;
  virtual void ListMapPoints(MapPointSet & out_map_points) const = 0;

  size_t Id() {return id_;}

  TimePoint GetTimeCreated() const { return time_point_; }
  const features::IFeatureExtractor * GetFeatureExtractor() const { return feature_extractor_; }
  const std::string & GetFilename() const { return filename_; }
  const features::BowVocabulary * GetVocabulary() const { return vocabulary_; }
 protected:
  const TimePoint time_point_;
  const std::string filename_;
  const features::IFeatureExtractor * feature_extractor_;
  const features::BowVocabulary * vocabulary_;
  size_t id_;
};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_BASE_FRAME_H_
