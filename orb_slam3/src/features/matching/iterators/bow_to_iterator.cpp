//
// Created by vahagn on 14/04/2021.
//

#include "features/matching/iterators/bow_to_iterator.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

BowToIterator & BowToIterator::operator++() {
  ++it_;
  // If to_map_points_exist_ is true, we should skip all
  // ids that do not correspond to a map point.
  while (map_points_to_ && it_ != end_it_
      && (to_map_points_exist_ ^ (map_points_to_->find(*it_) != map_points_to_->end())))
    ++it_;

  if (it_ != end_it_) {
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
  while (bow_it_to_ != bow_end_to_ && bow_it_from_ != bow_end_from_) {

    if (bow_it_to_->first == bow_it_from_->first) {
      it_ = bow_it_to_->second.begin();
      end_it_ = bow_it_to_->second.end();
      while (map_points_to_ && it_ != end_it_
          && (to_map_points_exist_ ^ (map_points_to_->find(*it_) != map_points_to_->end())))
        ++it_;
      if(it_ == end_it_)
        continue;
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