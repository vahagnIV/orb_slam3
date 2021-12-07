//
// Created by vahagn on 05.12.21.
//

#ifndef ORB_SLAM3_DRAWER_OBJECTS_BUFFER_H_
#define ORB_SLAM3_DRAWER_OBJECTS_BUFFER_H_

#include <stack>
#include <GL/glew.h>

namespace orb_slam3 {
namespace drawer {

class ObjectsBuffer {
 public:
  ObjectsBuffer(size_t count) : size_(count) {
    glGenBuffers(1, &buffer_id);
  }

  bool Empty() const {
    return free_offsets_.empty();
  }

 private:
  GLuint buffer_id;
  std::stack<size_t> free_offsets_;
  size_t size_;
  size_t carrette_position_;

};

}
}
#endif //ORB_SLAM3_DRAWER_OBJECTS_BUFFER_H_
