//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_FRAME_H_
#include "base_frame.h"
#include <frame/map_point_visibility_params.h>

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
  virtual bool FindMapPointsFromReferenceKeyFrame(const KeyFrame * reference_keyframe) = 0;
  virtual bool EstimatePositionByProjectingMapPoints(Frame * frame,
                                                     std::list<MapPointVisibilityParams> & out_visibles) = 0;
  virtual KeyFrame * CreateKeyFrame() = 0;
  virtual void OptimizePose() = 0;
  virtual size_t GetMapPointCount() const = 0;
  virtual void FilterVisibleMapPoints(const std::unordered_set<map::MapPoint *> & map_points,
                              std::list<MapPointVisibilityParams> & out_filtered_map_points,
                              precision_t radius_multiplier = 1,
                              unsigned fixed_window_size = 0) const = 0;

  virtual void SearchInVisiblePoints(const std::list<MapPointVisibilityParams> & filtered_map_points) = 0;
  virtual void UpdateFromReferenceKeyFrame() = 0;
  virtual void SearchWordSharingKeyFrames(const std::vector<std::unordered_set<KeyFrame*>> & inverted_file,
                                          std::list<KeyFrame *> & out_word_sharing_key_frames) = 0;
 private:
  static size_t next_id_;


};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_FRAME_H_
