//
// Created by vahagn on 08.05.21.
//

// === g2o ===
#include <g2o/core/robust_kernel_impl.h>
// === orb_slam3 ===
#include <features/features.h>
#include <optimization/edges/ba_binary_edge.h>
#include <optimization/edges/ba_unary_edge.h>

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_OBSERVATION_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_OBSERVATION_H_
namespace orb_slam3 {

namespace map {
class MapPoint;
}

namespace frame {
class KeyFrame;

class Observation  {
 public:
  Observation(map::MapPoint * map_point, KeyFrame * key_frame, size_t feature_ids) ;
  Observation(map::MapPoint * map_point, KeyFrame * key_frame, size_t feature_id_left, size_t feature_id_right) ;

  virtual ~Observation() = default;
  /*!
   * Creates a g2o edge for bundle adjustment
   * @return g2o edge with set vertices
   */
  optimization::edges::BABinaryEdge * CreateBinaryEdge() const;

  /*!
   * Create a g2o edge for pose optimization
   * @return g2o edge with set vertex
   */
  optimization::edges::BAUnaryEdge *CreateEdge() const;

  /*!
   * Getter for the corresponding frame
   * @return The frame base where the map point was observed
   */
  virtual KeyFrame * GetKeyFrame() const { return key_frame_; };

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
  virtual void AppendDescriptorsToList(std::vector<features::DescriptorType> & out_descriptor_ptr) const;
  const std::vector<std::size_t> GetFeatutreIds() const;

  /*!
   * Creates a robust kernel for optimization
   * @return The pointer for the newly created kernel
   */
  virtual g2o::RobustKernel * CreateRobustKernel();

 protected:
  bool IsMonocular() const;
  map::MapPoint * map_point_;
  KeyFrame * key_frame_;
  const std::vector<std::size_t> feature_ids_;

};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_OBSERVATION_H_
