//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_FRAME_H_
#include "base_frame.h"

namespace orb_slam3 {
namespace map {
class MapPoint;
}

namespace frame {
class KeyFrame;

class Frame : public BaseFrame {
 public:
  Frame(TimePoint & time_point,
        const std::string & filename,
        const features::IFeatureExtractor * feature_extractor,
        const features::BowVocabulary * vocabulary,
        const SensorConstants * sensor_constants) : BaseFrame(time_point,
                                                                filename,
                                                                feature_extractor,
                                                                vocabulary, sensor_constants, ++next_id_) {}

  virtual ~Frame() = default;
 public:
  virtual bool IsValid() const = 0;
  virtual bool Link(Frame * other) = 0;
  virtual bool EstimatePositionFromReferenceKeyframe(const KeyFrame * reference_keyframe) = 0;
  virtual bool EstimatePositionByProjectingMapPoints(const MapPointSet & map_points) = 0;
  virtual KeyFrame * CreateKeyFrame() = 0;
 private:
  static size_t next_id_;


};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_FRAME_H_