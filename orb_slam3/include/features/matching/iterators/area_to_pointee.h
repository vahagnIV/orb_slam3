//
// Created by vahagn on 12/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_AREA_TO_POINTEE_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_AREA_TO_POINTEE_H_

// === orb_slam3 ===
#include <typedefs.h>
#include <features/features.h>
#include "vector_from_pointee.h"
#include "vector_from_iterator.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

class AreaToIterator;

class AreaToPointee : public VectorFromPointee {
  friend class AreaToIterator;
 public:
  typedef VectorFromIterator<std::size_t> iterator;
  AreaToPointee() = default;
  AreaToPointee(VectorFromPointee::id_type id, Features *features_to, Features *features_from, size_t window_size)
      : VectorFromPointee(&features_to->descriptors, id),
        features_to_(features_to),
        features_from_(features_from),
        window_size_(window_size) {
    InitializeIterators();
  }
  iterator begin() {
    return begin_iterator_;
  }
  iterator end() {
    return end_iterator_;
  }
 private:
  void InitializeIterators() {
    features_from_->ListFeaturesInArea(features_to_->keypoints[id_].X(),
                                       features_to_->keypoints[id_].Y(),
                                       window_size_,
                                       0,
                                       0,
                                       from_indices_);
    end_iterator_ = iterator(from_indices_.end(), from_indices_.end(), nullptr);
    begin_iterator_ = iterator(from_indices_.begin(), from_indices_.end(), &features_from_->descriptors);
  }
  Features *features_to_;
  Features *features_from_;
  size_t window_size_;
  std::vector<std::size_t> from_indices_;
  iterator end_iterator_;
  iterator begin_iterator_;

};

}
}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_AREA_TO_POINTEE_H_
