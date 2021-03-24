//
// Created by vahagn on 24/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_ITERATORS_BOW_ITERATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_ITERATORS_BOW_ITERATOR_H_

#include <unordered_set>

#include "idescriptor_iterator.h"
#include <features/features.h>

namespace orb_slam3 {
namespace features {
namespace iterators {

class FeatureVectorTraverseIterator {
  friend class BowIterator;
 public:
  FeatureVectorTraverseIterator &operator++();
  bool operator==(const FeatureVectorTraverseIterator &other);
  bool operator!=(const FeatureVectorTraverseIterator &other);
  size_t operator*() const;
 private:
  FeatureVectorTraverseIterator(const std::vector<unsigned> &vectoto_iterate);
  FeatureVectorTraverseIterator(std::vector<unsigned>::const_iterator iterator);
  FeatureVectorTraverseIterator() = default;
 private:
  std::vector<unsigned>::const_iterator vector_it_;
  const std::vector<unsigned> * vector_to_iterate_ptr_;
};

class BowIterator : public IJointDescriptorIterator<FeatureVectorTraverseIterator> {
 public:
  BowIterator(const DBoW2::FeatureVector &vector_to,
              const DBoW2::FeatureVector &vector_from);
  BowIterator &operator++() override;
  size_t IdxTo() override;
  FeatureVectorTraverseIterator begin() override;
  FeatureVectorTraverseIterator end() override;
  bool IsValid() override;

 private:
  void AdvanceEqualizeNodes();
 private:
  const DBoW2::FeatureVector *to_feature_vec_ptr_;
  const DBoW2::FeatureVector *from_feature_vec_ptr_;
  DBoW2::FeatureVector::const_iterator to_it;
  DBoW2::FeatureVector::const_iterator from_it;
  FeatureVectorTraverseIterator to_traversal_iterator_;
  FeatureVectorTraverseIterator to_traversal_iterator_end_;
  FeatureVectorTraverseIterator from_traversal_iterator_end_;
};

}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_ITERATORS_BOW_ITERATOR_H_
