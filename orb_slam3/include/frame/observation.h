//
// Created by vahagn on 08.05.21.
//

// === g2o ===
#include <g2o/core/base_edge.h>

// === orb_slam3 ===
#include <frame/frame_base.h>
#include <identifiable.h>

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_OBSERVATION_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_OBSERVATION_H_
namespace orb_slam3 {
namespace frame {

class Observation : public Identifiable {
 public:
  /*!
   * Creates a g2o edge for bundle adjustment
   * @return g2o edge with set vertices
   */
  virtual g2o::BaseEdge<2, Eigen::Vector2d> * CreateMultiEdge() = 0;

  /*!
   * Create a g2o edge for pose optimization
   * @return g2o edge with set vertices
   */
  virtual g2o::BaseEdge<2, Eigen::Vector2d> * CreateEdge() = 0;

  /*!
   * Getter for the corresponding frame
   * @return The frame base where the map point was observed
   */
  virtual FrameBase * GetFrame() = 0;

  /*!
   * Appends the descriptors that correspond to the map_point and frame to the list
   * @param feature_id The id of the keypoint
   * @param out_descriptor_ptr The vector to which the descriptors will be appended
   */
  virtual void AppendDescriptorsToList(std::vector<features::DescriptorType> & out_descriptor_ptr) const = 0;
  virtual ~Observation() = default;
};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_OBSERVATION_H_
