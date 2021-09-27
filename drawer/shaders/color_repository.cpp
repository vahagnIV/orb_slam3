//
// Created by vahagn on 18.08.21.
//

#include "color_repository.h"

namespace orb_slam3 {
namespace drawer {

const float ColorRepository::red[3] = {1, 0, 0};
const float ColorRepository::green[3] = {0, 1, 0};
const float ColorRepository::blue[3] = {0, 0, 1};

const float *ColorRepository::Red() {
  return red ;
}

const float *ColorRepository::Green() {
  return green;
}

const float *ColorRepository::Blue() {
  return blue;
}

}
}