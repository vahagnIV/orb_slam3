//
// Created by vahagn on 23/03/2021.
//
#include <features/bow_container.h>
namespace orb_slam3 {
namespace features {

BowContainer::BowContainer() : end_iterator_(this->feature_vector, this->feature_vector) {
  end_iterator_.to_it = this->feature_vector.end();
}

void BowContainer::ComputeBow(const vector<cv::Mat> &descriptors) {
  vocabulary->transform(descriptors, bow_vector, feature_vector, 4);
}

BowContainer::iterator::iterator(const DBoW2::FeatureVector &first, const DBoW2::FeatureVector &second)
    : to_feature_vec_ptr_(&first),
      from_feature_vec_ptr_(&second),
      to_it(first.begin()), from_it(second.begin()) {
  AdvanceEquilize();

}

BowContainer::iterator &BowContainer::iterator::operator++() {
  from_it++;
  to_it++;
  AdvanceEquilize();
  return *this;
}

BowContainer::iterator BowContainer::iterator::operator++(int) {
  ++(*this);
  return *this;
}

bool BowContainer::iterator::operator==(const iterator &other) const {
  if (to_it == this->from_feature_vec_ptr_->end() && other.to_it == other.from_feature_vec_ptr_->end())
    return true;
  return to_it == other.to_it && from_it == from_it;
}

bool BowContainer::iterator::operator!=(const BowContainer::iterator &other) const {
  return !(*this == other);
}

const std::vector<unsigned> &BowContainer::iterator::ToIdx() {
  return to_it->second;
}

const std::vector<unsigned> &BowContainer::iterator::FromIdx() {
  return from_it->second;
}

void BowContainer::iterator::SetToEnd() {
  to_it = to_feature_vec_ptr_->end();
  from_it = from_feature_vec_ptr_->end();
}

void BowContainer::iterator::AdvanceEquilize() {
  while (true) {
    if (to_it == to_feature_vec_ptr_->end() || from_it == from_feature_vec_ptr_->end()) {
      SetToEnd();
      return;
    }

    if (to_it->first == from_it->first) {
      return;
    }

    if (to_it->first > from_it->first) {
      from_it = from_feature_vec_ptr_->lower_bound(to_it->first);
    } else
      to_it = to_feature_vec_ptr_->lower_bound(from_it->first);
  }

}

BowContainer::iterator BowContainer::Begin(const BowContainer &other) const {
  return BowContainer::iterator(this->feature_vector, other.feature_vector);
}

BowContainer::iterator BowContainer::End() const {
  return end_iterator_;
}

}
}