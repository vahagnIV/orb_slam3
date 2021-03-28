#ifndef ORB_SLAM3_INCLUDE_MAP_MAP_POINT_H_
#define ORB_SLAM3_INCLUDE_MAP_MAP_POINT_H_

// == orb-slam3 ===
#include <typedefs.h>

namespace orb_slam3 {
namespace features {

struct KeyPoint {
  const precision_t & X() const { return pt[0]; }
  const precision_t & Y() const { return pt[1]; }
  precision_t & X() { return pt[0]; }
  precision_t & Y() { return pt[1]; }
  precision_t angle;
  TPoint2D pt;
  precision_t size;
  unsigned level;
};

}  // namespace map
}  // namespace orb_slam3

#endif  // ORB_SLAM3_INCLUDE_MAP_MAP_POINT_H_