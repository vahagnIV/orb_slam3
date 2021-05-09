//
// Created by vahagn on 08.05.21.
//

// === g2o ===
#include <g2o/core/base_edge.h>
#include <g2o/core/robust_kernel_impl.h>
// === orb_slam3 ===
#include <identifiable.h>
#include <features/features.h>

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_OBSERVATION_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_OBSERVATION_H_
namespace orb_slam3 {

namespace map {
class MapPoint;
}

namespace frame {
class FrameBase;

class Observation : public Identifiable {
 public:
  Observation(map::MapPoint * map_point) : map_point_(map_point) {}
  /*!
   * Creates a g2o edge for bundle adjustment
   * @return g2o edge with set vertices
   */
  virtual g2o::BaseEdge<2, Eigen::Vector2d> * CreateBinaryEdge() = 0;

  /*!
   * Create a g2o edge for pose optimization
   * @return g2o edge with set vertex
   */
  virtual g2o::BaseEdge<2, Eigen::Vector2d> * CreateEdge() = 0;

  /*!
   * Getter for the corresponding frame
   * @return The frame base where the map point was observed
   */
  virtual FrameBase * GetFrame() = 0;

  /*!
   * Getter for the map point
   * @return
   */
  map::MapPoint * GetMapPoint() { return map_point_; }

  /*!
   * Appends the descriptors that correspond to the map_point and frame to the list
   * @param feature_id The id of the keypoint
   * @param out_descriptor_ptr The vector to which the descriptors will be appended
   */
  virtual void AppendDescriptorsToList(std::vector<features::DescriptorType> & out_descriptor_ptr) const = 0;

  /*!
   * Creates a robust kernel for optimization
   * @return The pointer for the newly created kernel
   */
  virtual g2o::RobustKernel * CreateRobustKernel() const = 0;
  virtual ~Observation() = default;
 protected:
  map::MapPoint * map_point_;

};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_OBSERVATION_H_
