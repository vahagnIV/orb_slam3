#ifndef ORB_SLAM3_INCLUDE_MAP_MAP_POINT_H_
#define ORB_SLAM3_INCLUDE_MAP_MAP_POINT_H_

// == orb-slam3 ===
#include "../typedefs.h"

namespace orb_slam3 {
namespace features {

struct KeyPoint {
  friend std::ostream & operator<<(std::ostream & stream, const KeyPoint & frame);
  KeyPoint() = default;
  KeyPoint(TPoint2D point, unsigned level = 0, precision_t angle = 0, precision_t size = 0)
      : pt(point), angle(angle), size(size), level(level) {}
  const precision_t & X() const { return pt.x(); }
  const precision_t & Y() const { return pt.y(); }
  precision_t & X() { return pt[0]; }
  precision_t & Y() { return pt[1]; }
  TPoint2D pt;
  precision_t angle;
  precision_t size;
  int level;
};

}  // namespace map
}  // namespace orb_slam3

#endif  // ORB_SLAM3_INCLUDE_MAP_MAP_POINT_H_