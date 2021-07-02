//
// Created by vahagn on 14/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_TO_ITERATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_TO_ITERATOR_H_

// === DBoW2 ===
#include <DBoW2/FeatureVector.h>

// === orb_slam3 ====
#include "bow_to_pointee.h"

namespace orb_slam3 {
namespace features {
namespace handlers {
namespace iterators {

class BowToIterator {
 public:
  typedef BowToPointee value_type;
  BowToIterator(
      DBoW2::FeatureVector::const_iterator begin,
      const DBoW2::FeatureVector * feature_vector_to,
      const DBoW2::FeatureVector * feature_vector_from,
      const features::Features * features_to,
      const features::Features * features_from) : bow_it_to_(begin),
                                                  bow_end_to_(feature_vector_to->end()),
                                                  bow_it_from_(feature_vector_from->begin()),
                                                  bow_end_from_(feature_vector_from->end()),
                                                  feature_vector_to_(feature_vector_to),
                                                  feature_vector_from_(feature_vector_from),
                                                  pointee_(feature_vector_from,
                                                           features_to,
                                                           features_from) {
    AdvanceUntilSameNode();
  }

  const BowToPointee & operator*() const { return pointee_; }
  BowToPointee & operator*() { return pointee_; }
  const BowToPointee * operator->() const { return &pointee_; }
  BowToPointee * operator->() { return &pointee_; }
  BowToIterator & operator++();

  friend bool operator==(const BowToIterator & a, const BowToIterator & b) {
    if ((a.bow_it_to_ == a.bow_end_to_ || a.bow_it_from_ == a.bow_end_from_)
        && (b.bow_it_to_ == b.bow_end_to_ || b.bow_it_from_ == b.bow_end_from_))
      return true;
    if ((a.bow_it_to_ == a.bow_end_to_ || a.bow_it_from_ == a.bow_end_from_)
        || (b.bow_it_to_ == b.bow_end_to_ || b.bow_it_from_ == b.bow_end_from_)) {
      return false;
    }
    return a.bow_it_to_ == b.bow_it_to_
        && a.bow_end_from_ == b.bow_it_from_
        && a.it_ == b.it_;
  }
  friend bool operator!=(const BowToIterator & a, const BowToIterator & b) {
    return !(a == b);
  }
 private:
  void AdvanceUntilSameNode();
 private:

  DBoW2::FeatureVector::const_iterator bow_it_to_;
  DBoW2::FeatureVector::const_iterator bow_end_to_;

  DBoW2::FeatureVector::const_iterator bow_it_from_;
  DBoW2::FeatureVector::const_iterator bow_end_from_;

  std::vector<unsigned>::const_iterator it_;
  std::vector<unsigned>::const_iterator end_it_;

  const DBoW2::FeatureVector * feature_vector_to_;
  const DBoW2::FeatureVector * feature_vector_from_;
  BowToPointee pointee_;
};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_TO_ITERATOR_H_
