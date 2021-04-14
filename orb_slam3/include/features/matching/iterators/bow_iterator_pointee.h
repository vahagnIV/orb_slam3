//
// Created by vahagn on 14/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_ITERATOR_POINTEE_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_ITERATOR_POINTEE_H_
#include <cstddef>

// === orb_slam3 ===
#include <features/features.h>
#include <DBoW2/FeatureVector.h>
#include "vector_from_iterator.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

class BowIteratorPointee {
 public:
  typedef size_t id_type;
  typedef VectorFromIterator<unsigned> iterator;
  BowIteratorPointee(DBoW2::FeatureVector *feature_vector_to,
                     DBoW2::FeatureVector *feature_vector_from,
                     features::Features *features_to,
                     features::Features *features_from) :
      features_to_(features_to),
      features_from_(features_from),
      feature_vector_to_(feature_vector_to),
      feature_vector_from_(feature_vector_from) {}

  id_type GetId() const { return id_; }
  iterator begin() { return begin_iterator_; }
  iterator end() { return end_iterator_; }
  const DescriptorType GetDescriptor() const {
    return features_to_->descriptors.row(feature_vector_to_->at(bow_id_).at(id_));
  }

  void SetBowId(size_t bow_id) {
    begin_iterator_ = iterator(feature_vector_from_->at(bow_id).begin(),
                               feature_vector_from_->at(bow_id).end(),
                               &features_from_->descriptors);
    end_iterator_ = iterator(feature_vector_from_->at(bow_id).end(),
                             feature_vector_from_->at(bow_id).end(),
                             &features_from_->descriptors);
  }

  void SetId(id_type id) {
    id_ = id;
  }
 private:
  size_t id_;
  size_t bow_id_;
  features::Features *features_to_;
  features::Features *features_from_;
  DBoW2::FeatureVector *feature_vector_to_;
  DBoW2::FeatureVector *feature_vector_from_;
  iterator begin_iterator_;
  iterator end_iterator_;

};

}
}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_ITERATOR_POINTEE_H_
