//
// Created by vahagn on 09/06/2021.
//

#include "key_point.h"
namespace orb_slam3 {
namespace features {

std::ostream & operator<<(std::ostream & stream, const KeyPoint & key_point) {
  WRITE_TO_STREAM(key_point.level, stream);
  WRITE_TO_STREAM(key_point.size, stream);
  WRITE_TO_STREAM(key_point.angle, stream);
  stream.write((char *) key_point.pt.data(), key_point.pt.rows() * sizeof(decltype(key_point.pt)::Scalar));
  return stream;
}

}
}