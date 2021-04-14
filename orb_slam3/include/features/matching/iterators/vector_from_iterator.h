//
// Created by vahagn on 12/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_VECTOR_FROM_ITERATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_VECTOR_FROM_ITERATOR_H_

#include <map>

// === orb_slam3 ===
#include <typedefs.h>
#include <features/features.h>
#include <map/map_point.h>
#include "vect_from_pointee.h"


namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

template<typename IdType>
class VectorFromIterator {
 public:
  typedef VectorFromPointee value_type;
  VectorFromIterator() : pointee_(nullptr, 0) {}
  VectorFromIterator(typename std::vector<IdType>::iterator begin,
                     typename std::vector<IdType>::iterator end,
                     const DescriptorSet *descriptors,
                     const std::map<std::size_t , map::MapPoint *> * map_points = nullptr,
                     bool require_exists = false) : pointee_(descriptors, *begin), it_(begin), end_(end) {
  }
  const VectorFromPointee & operator*() const { return pointee_; }
  VectorFromPointee & operator*() { return pointee_; }
  const VectorFromPointee *operator->() const { return &pointee_; }
  VectorFromPointee *operator->() { return &pointee_; }
  VectorFromIterator & operator++() {
    if(it_ != end_ && ++it_ != end_ )
      pointee_.SetId( *it_);
    return *this;
  }
  friend bool operator==(const VectorFromIterator & a, const VectorFromIterator & b) { return a.it_ == b.it_; }
  friend bool operator!=(const VectorFromIterator & a, const VectorFromIterator & b) { return a.it_ != b.it_; }
 private:
  VectorFromPointee pointee_;
  typename std::vector<IdType>::iterator it_;
  typename std::vector<IdType>::iterator end_;
};
}
}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_VECTOR_FROM_ITERATOR_H_
