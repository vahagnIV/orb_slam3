//
// Created by vahagn on 26/02/21.
//

#include "features/feature_utils.h"
namespace orb_slam3 {
namespace features {

int DescriptorDistance(const DescriptorType & d1,
                       const DescriptorType & d2) {
  assert(d1.cols() == d2.cols());
  const int32_t * pa = reinterpret_cast<const int32_t * const>(d1.data());
  const int32_t * pb = reinterpret_cast<const int32_t * const>(d2.data());

  unsigned dist = 0;

  for (size_t i = 0; i < d1.cols() >> 2; ++i, ++pa, ++pb) {
    int v = *pa ^*pb;
    v -= ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}

}
}