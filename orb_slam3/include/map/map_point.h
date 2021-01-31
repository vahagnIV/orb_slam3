#ifndef ORB_SLAM3_INCLUDE_MAP_MAP_POINT_H_
#define ORB_SLAM3_INCLUDE_MAP_MAP_POINT_H_

#include <typedefs.h>

namespace orb_slam3 {
namespace map {

class MapPoint {
 public:
  MapPoint() = default;
  void SetAngle(precision_t angle) noexcept{
      angle_  = angle;
  }
  
  void SetX(precision_t x) noexcept{
      x_ = x;
  }

  void SetY(precision_t y) noexcept{
      y_ = y;
  }

  precision_t X() const noexcept {return x_;}
  precision_t Y() const noexcept {return y_;}
  precision_t Angle() const noexcept {return angle_;}

private:
  precision_t angle_;
  precision_t x_;
  precision_t y_;
};

}  // namespace map
}  // namespace orb_slam3

#endif  // ORB_SLAM3_INCLUDE_MAP_MAP_POINT_H_