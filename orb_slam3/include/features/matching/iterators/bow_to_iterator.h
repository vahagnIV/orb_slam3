//
// Created by vahagn on 14/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_TO_ITERATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_TO_ITERATOR_H_

// === DBoW2 ===
#include <DBoW2/FeatureVector.h>

// === orb_slam3 ====
#include "bow_iterator_pointee.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

class BowToIterator {
 public:
  typedef BowIteratorPointee value_type;
  BowToIterator(
      DBoW2::FeatureVector::iterator begin,
      DBoW2::FeatureVector *feature_vector_to,
      DBoW2::FeatureVector *feature_vector_from,
      features::Features *features_to,
      features::Features *features_from,
      std::map<std::size_t, map::MapPoint *> *map_points_to = nullptr,
      std::map<std::size_t, map::MapPoint *> *map_points_from = nullptr,
      bool to_map_points_exist = false,
      bool from_map_points_exist = false) :
      bow_it_to_(begin),
      bow_end_to_(feature_vector_to->end()),
      bow_it_from_(feature_vector_from->begin()),
      bow_end_from_(feature_vector_from->end()),
      feature_vector_to_(feature_vector_to),
      feature_vector_from_(feature_vector_from),
      pointee_(feature_vector_to,
               feature_vector_from,
               features_to,
               features_from,
               map_points_from,
               from_map_points_exist),
      map_points_to_(map_points_to),
      to_map_points_exist_(to_map_points_exist) {
    AdvanceUntilSameNode();
  }

  const BowIteratorPointee & operator*() const { return pointee_; }
  BowIteratorPointee & operator*() { return pointee_; }
  const BowIteratorPointee *operator->() const { return &pointee_; }
  BowIteratorPointee *operator->() { return &pointee_; }
  BowToIterator & operator++();

  friend bool operator==(const BowToIterator & a, const BowToIterator & b) {
    return a.bow_it_to_ == b.bow_it_to_;
  }
  friend bool operator!=(const BowToIterator & a, const BowToIterator & b) {
    return  a.bow_it_to_ != b.bow_it_to_;
  }
 private:
  void AdvanceUntilSameNode();
 private:

  DBoW2::FeatureVector::iterator bow_it_to_;
  DBoW2::FeatureVector::iterator bow_end_to_;

  DBoW2::FeatureVector::iterator bow_it_from_;
  DBoW2::FeatureVector::iterator bow_end_from_;

  std::vector<unsigned>::iterator it_;
  std::vector<unsigned>::iterator end_it_;

  DBoW2::FeatureVector *feature_vector_to_;
  DBoW2::FeatureVector *feature_vector_from_;
  BowIteratorPointee pointee_;

  std::map<std::size_t, map::MapPoint *> *map_points_to_;
  bool to_map_points_exist_;

};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_TO_ITERATOR_H_
