//
// Created by vahagn on 08.05.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_MONOCULAR_OBSERVATION_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_MONOCULAR_OBSERVATION_H_

// === orb_slam3 ===
#include <frame/observation.h>
#include <frame/monocular_frame.h>
namespace orb_slam3 {
namespace frame {

class MonocularObservation : public Observation {
 public:

  g2o::BaseEdge<2, Eigen::Vector2d> * CreateMultiEdge() override;
  // Observation iface
  MonocularObservation(MonocularFrame * frame, size_t feature_id);
  FrameBase * GetFrame() override { return frame_; }
  void AppendDescriptorsToList(vector<features::DescriptorType> & out_descriptor_ptr) const override;
  g2o::BaseEdge<2, Eigen::Vector2d> * CreateEdge() override;

  // Monocular
  size_t GetFeatureId() const { return feature_id_; }
 private:
  MonocularFrame * frame_;
  size_t feature_id_;

};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_MONOCULAR_OBSERVATION_H_
