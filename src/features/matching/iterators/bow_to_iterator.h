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
namespace matching {
namespace iterators {

class BowToIterator {
 public:
  typedef BowToPointee value_type;
  BowToIterator(
      DBoW2::FeatureVector::const_iterator begin,
      const DBoW2::FeatureVector * feature_vector_to,
      const DBoW2::FeatureVector * feature_vector_from,
      const features::Features * features_to,
      const features::Features * features_from,
      const std::map<std::size_t, map::MapPoint *> * map_points_to = nullptr,
      const std::map<std::size_t, map::MapPoint *> * map_points_from = nullptr,
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

  const BowToPointee & operator*() const { return pointee_; }
  BowToPointee & operator*() { return pointee_; }
  const BowToPointee * operator->() const { return &pointee_; }
  BowToPointee * operator->() { return &pointee_; }
  BowToIterator & operator++();

  // TODO: implement correctly
  friend bool operator==(const BowToIterator & a, const BowToIterator & b) {
    return a.bow_it_to_ == b.bow_it_to_;
  }
  friend bool operator!=(const BowToIterator & a, const BowToIterator & b) {
    return a.bow_it_to_ != b.bow_it_to_;
  }
 private:
  void AdvanceUntilSameNode();
  void AdvanceUntilValidIterator();
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

  const std::map<std::size_t, map::MapPoint *> * map_points_to_;
  bool to_map_points_exist_;

};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_TO_ITERATOR_H_
