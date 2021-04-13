//
// Created by vahagn on 12/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_AREA_FROM_ITERATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_AREA_FROM_ITERATOR_H_

// === orb_slam3 ===
#include <typedefs.h>
#include <features/features.h>
#include "area_from_pointee.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

class AreaFromIterator {
 public:
  typedef AreaFromPointee value_type;
  AreaFromIterator():pointee_(nullptr, 0) {}
  AreaFromIterator(std::vector<std::size_t>::iterator it, DescriptorSet * descriptors) : pointee_(descriptors, *it) {
  }
  const AreaFromPointee & operator*() const { return pointee_; }
  AreaFromPointee & operator*() { return pointee_; }
  const AreaFromPointee *operator->() const { return &pointee_; }
  AreaFromPointee *operator->() { return &pointee_; }
  AreaFromIterator & operator++() {
    ++it_;
    pointee_.id_ = *it_;
    return *this;
  }
  friend bool operator==(const AreaFromIterator & a, const AreaFromIterator & b) { return a.it_ == b.it_; }
  friend bool operator!=(const AreaFromIterator & a, const AreaFromIterator & b) { return a.it_ != b.it_; }
 private:
  AreaFromPointee pointee_;
  std::vector<std::size_t>::iterator it_;
};
}
}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_AREA_FROM_ITERATOR_H_
