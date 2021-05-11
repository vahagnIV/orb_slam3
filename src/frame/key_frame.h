//
// Created by vahagn on 11/05/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_KEY_FRAME_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_KEY_FRAME_H_
#include "base_frame.h"
#include <features/features.h>
#include <frame/covisibility_graph_node.h>

namespace orb_slam3 {
namespace frame {

class KeyFrame : public BaseFrame {
 public:
  KeyFrame(TimePoint time_point,
           const std::string & filename,
           const features::IFeatureExtractor * feature_extractor,
           const features::BowVocabulary * vocabulary,
           size_t id) : BaseFrame(time_point, filename, feature_extractor, vocabulary, id),
                                                         covisibility_graph_(this) {}

  virtual ~KeyFrame() = default;
 public:
  virtual TVector3D GetNormal(const TPoint3D & point) const = 0;
  CovisibilityGraphNode & GetCovisibilityGraph() {
    return covisibility_graph_;
  }
 protected:
  CovisibilityGraphNode covisibility_graph_;

};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_KEY_FRAME_H_
