//
// Created by vahagn on 16/08/2021.
//

#ifndef ORB_SLAM3_DRAWER_KEY_FRAME_DETAILS_H_
#define ORB_SLAM3_DRAWER_KEY_FRAME_DETAILS_H_

#include <typedefs.h>

namespace orb_slam3 {
namespace drawer {
struct KeyFrameDetails {
  size_t id;
  size_t map_id;
  TVector3D::Scalar vertices[18];
  GLuint vertex_buffer_id;
};

}
}
#endif //ORB_SLAM3_DRAWER_KEY_FRAME_DETAILS_H_
