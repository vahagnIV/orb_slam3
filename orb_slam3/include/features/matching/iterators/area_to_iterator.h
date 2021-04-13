//
// Created by vahagn on 12/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_AREA_TO_ITERATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_AREA_TO_ITERATOR_H_
#include "area_to_pointee.h"
namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

class AreaToIterator {
 public:
  typedef size_t ToIdType;
  typedef size_t FromIdType;
  typedef AreaToPointee value_type;
  AreaToIterator() = default;
  AreaToIterator(ToIdType id, Features * features_to, Features * features_from, size_t window_size)
      : pointee_(id, features_to, features_from, window_size), descriptor_count_(features_to->Size()) {
    pointee_.id_ = std::min(descriptor_count_, pointee_.id_);
  }

  const AreaToPointee & operator*() const { return pointee_; }
  AreaToPointee & operator*() { return pointee_; }
  const AreaToPointee * operator->() const { return &pointee_; }
  AreaToPointee * operator->() { return &pointee_; }
  AreaToIterator & operator++() {
    pointee_.id_ = std::min(descriptor_count_, pointee_.id_ + 1);
    return *this;
  }
  friend bool operator==(const AreaToIterator & a, const AreaToIterator & b) {
    return a.pointee_.GetId() == b.pointee_.GetId();
  }
  friend bool operator!=(const AreaToIterator & a, const AreaToIterator & b) {
    return a.pointee_.GetId() != b.pointee_.GetId();
  }
 private:
  AreaToPointee pointee_;
  size_t descriptor_count_;

};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_AREA_TO_ITERATOR_H_
