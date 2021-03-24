//
// Created by vahagn on 24/03/2021.
//

#include "features/iterators/area_iterator.h"

namespace orb_slam3 {
namespace features {
namespace iterators {

AreaIterator::AreaIterator(const Features &features_from, const Features &features_to, size_t window_size)
    : to_idx_(0),
      features_from_(&features_from),
      features_to_(&features_to),
      window_size_(window_size) {
  if (IsValid())
    features_from_->ListFeaturesInArea(features_to.keypoints[to_idx_].X(),
                                       features_to.keypoints[to_idx_].Y(),
                                       window_size_,
                                       0,
                                       0,
                                       selected_features_);
}

AreaIterator &AreaIterator::operator++() {

  ++to_idx_;
  if (IsValid()) {
    features_from_->ListFeaturesInArea(features_to_->keypoints[to_idx_].X(),
                                       features_to_->keypoints[to_idx_].Y(),
                                       window_size_,
                                       0,
                                       0,
                                       selected_features_);
  }

  return *this;
}

size_t AreaIterator::IdxTo() {
  return to_idx_;
}

bool AreaIterator::IsValid() {
  return to_idx_ < features_to_->Size() - 1;
}

vector<size_t>::iterator AreaIterator::begin() {
  return selected_features_.begin();
}

vector<size_t>::iterator AreaIterator::end() {
  return selected_features_.end();
}

}
}
}