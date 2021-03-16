//
// Created by vahagn on 16/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_COVISIBILITY_CONTAINER_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_COVISIBILITY_CONTAINER_H_
#include <vector>

namespace orb_slam3 {
namespace frame {
class FrameBase;

struct CovisibilityContainer {
  std::map<FrameBase*, unsigned > connected_frame_weights;
  std::vectot<FrameBase*> connected_frames;
};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_COVISIBILITY_CONTAINER_H_
