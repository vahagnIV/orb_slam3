//
// Created by vahagn on 14/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_TO_POINTEE_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_TO_POINTEE_H_
#include <cstddef>

// === orb_slam3 ===
#include <features/features.h>
#include <DBoW2/FeatureVector.h>
#include <features/matching/iterators/vector_from_iterator.h>

namespace orb_slam3 {
namespace features {
namespace handlers {
namespace iterators {

class BowToPointee {
 public:
  typedef size_t id_type;
  typedef matching::iterators::VectorFromIterator<unsigned> iterator;
  BowToPointee(const DBoW2::FeatureVector * feature_vector_from,
               const features::Features * features_to,
               const features::Features * features_from) :
      features_to_(features_to),
      features_from_(features_from),
      feature_vector_from_(feature_vector_from){}

  id_type GetId() const { return id_; }
  iterator begin() { return begin_iterator_; }
  iterator end() { return end_iterator_; }
  const DescriptorType GetDescriptor() const {
    return features_to_->descriptors.row(id_);
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
  const features::Features * features_to_;
  const features::Features * features_from_;
  const DBoW2::FeatureVector * feature_vector_from_;
  iterator begin_iterator_;
  iterator end_iterator_;

};

}
}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_BOW_TO_POINTEE_H_
