//
// Created by vahagn on 24/03/2021.
//

#include "features/matching/iterators/bow_iterator.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

FeatureVectorTraverseIterator &FeatureVectorTraverseIterator::operator++() {
  ++vector_it_;
  return *this;
}

bool FeatureVectorTraverseIterator::operator==(const FeatureVectorTraverseIterator &other) {
  return vector_it_ == other.vector_it_;
}
bool FeatureVectorTraverseIterator::operator!=(const FeatureVectorTraverseIterator &other) {
  return vector_it_ != other.vector_it_;
}
size_t FeatureVectorTraverseIterator::operator*() const {
  return *vector_it_;
}

FeatureVectorTraverseIterator::FeatureVectorTraverseIterator(const vector<unsigned int> &vectoto_iterate)
    : vector_to_iterate_ptr_(&vectoto_iterate) {
  vector_it_ = vector_to_iterate_ptr_->begin();
}

FeatureVectorTraverseIterator::FeatureVectorTraverseIterator(vector<unsigned int>::const_iterator iterator)
    : vector_it_(iterator) {

}

BowIterator::BowIterator(const DBoW2::FeatureVector &vector_to, const DBoW2::FeatureVector &vector_from)
    : to_feature_vec_ptr_(&vector_to),
      from_feature_vec_ptr_(&vector_from) {
  to_it = to_feature_vec_ptr_->begin();
  from_it = from_feature_vec_ptr_->begin();
  AdvanceEqualizeNodes();

}

BowIterator &BowIterator::operator++() {
  if (to_traversal_iterator_ != to_traversal_iterator_end_)
    ++to_traversal_iterator_;
  if (to_traversal_iterator_ == to_traversal_iterator_end_) {
    to_it++;
    from_it++;
    AdvanceEqualizeNodes();
  }
  return *this;
}

size_t BowIterator::IdxTo() {
  return *to_traversal_iterator_;
}

FeatureVectorTraverseIterator BowIterator::begin() {
  return FeatureVectorTraverseIterator(from_it->second);
}

FeatureVectorTraverseIterator BowIterator::end() {
  return from_traversal_iterator_end_;
}

bool BowIterator::IsValid() {
  return to_it != to_feature_vec_ptr_->end();
}

void BowIterator::AdvanceEqualizeNodes() {
  while (true) {
    if (!IsValid()) {
      break;
    }

    if (to_it->first == from_it->first) {
      break;
    }

    if (to_it->first > from_it->first) {
      from_it = from_feature_vec_ptr_->lower_bound(to_it->first);
    } else
      to_it = to_feature_vec_ptr_->lower_bound(from_it->first);
  }

  if (IsValid()) {
    to_traversal_iterator_ = FeatureVectorTraverseIterator(to_it->second);
    to_traversal_iterator_end_ = FeatureVectorTraverseIterator(to_it->second.end());
    from_traversal_iterator_end_ = FeatureVectorTraverseIterator(from_it->second.end());
  }
}

}
}
}
}