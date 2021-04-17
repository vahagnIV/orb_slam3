//
// Created by vahagn on 13/04/2021.
//

#include "features/matching/iterators/projection_search_iterator.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

ProjectionSearchIterator::ProjectionSearchIterator(std::unordered_set<map::MapPoint *>::iterator begin,
                                                   std::unordered_set<map::MapPoint *>::iterator end,
                                                   const std::unordered_set<map::MapPoint *> * to_map_points,
                                                   const Features * from_features,
                                                   const geometry::Pose * pose,
                                                   const camera::MonocularCamera * camera,
                                                   const IFeatureExtractor * feature_extractor)
    : it_(begin),
      end_(end),
      to_map_points_(to_map_points),
      pose_(pose),
      camera_(camera), pointee_(from_features, feature_extractor) {
  AdvanceIteratorUntilGood();

}

ProjectionSearchIterator & ProjectionSearchIterator::operator++() {
  ++it_;
  AdvanceIteratorUntilGood();
  return *this;
}

void ProjectionSearchIterator::AdvanceIteratorUntilGood() {
  while (it_ != end_ && (to_map_points_->find(*it_) != to_map_points_->end() &&
      !pointee_.SetMapPointAndCompute(*it_,
                                      camera_,
                                      pose_)))
    ++it_;
  if (it_ != end_)
    pointee_.InitializeIterators();
}

}
}
}
}