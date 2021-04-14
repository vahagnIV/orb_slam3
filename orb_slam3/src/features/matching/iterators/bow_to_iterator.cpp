//
// Created by vahagn on 14/04/2021.
//

#include "features/matching/iterators/bow_to_iterator.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

BowToIterator & BowToIterator::operator++() {
  if (it_ != end_it_ && ++it_ != end_it_) {
    pointee_.SetId(*it_);
    return *this;
  }
  if (bow_it_to_ != bow_end_to_)
    ++bow_it_to_;
  if (bow_it_from_ != bow_end_from_)
    ++bow_it_from_;
  AdvanceUntilSameNode();
  return *this;
}

void BowToIterator::AdvanceUntilSameNode() {
  while (true) {
    if (bow_it_to_ == bow_end_to_ || bow_it_from_ == bow_end_from_)
      break;

    if (bow_it_to_->first == bow_it_from_->first) {
      it_ = bow_it_to_->second.begin();
      end_it_ = bow_it_to_->second.end();
      pointee_.SetBowId(bow_it_to_->first);
      pointee_.SetId(*it_);
      break;
    }

    if (bow_it_to_->first < bow_it_from_->first) {
      bow_it_to_ = feature_vector_to_->lower_bound(bow_it_from_->first);
    } else
      bow_it_from_ = feature_vector_from_->lower_bound(bow_it_to_->first);
  }
}

}
}
}
}