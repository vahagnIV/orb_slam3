//
// Created by vahagn on 18.08.21.
//

#ifndef ORB_SLAM3_DRAWER_SHADERS_COLOR_REPOSITORY_H_
#define ORB_SLAM3_DRAWER_SHADERS_COLOR_REPOSITORY_H_

namespace orb_slam3 {
namespace drawer {

class ColorRepository {
 public:
  static const float *Red();
  static const float *Green();
  static const float *Blue();
  static const float *Pink();
 private:
  static const float red[3];
  static const float green[3];
  static const float blue[3];
  static const float pink[3];

};

}
}
#endif //ORB_SLAM3_DRAWER_SHADERS_COLOR_REPOSITORY_H_
