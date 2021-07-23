//
// Created by vahagn on 13/04/2021.
//

#include "projection_search_iterator.h"

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

ProjectionSearchIterator::ProjectionSearchIterator(std::list<frame::MapPointVisibilityParams>::const_iterator begin,
                                                   std::list<frame::MapPointVisibilityParams>::const_iterator end,
                                                   const Features * from_features)
    : it_(begin),
      end_(end),
      pointee_(from_features) {
  if(begin != end)
    pointee_.SetVisibileMapPoint(*it_);
}

ProjectionSearchIterator & ProjectionSearchIterator::operator++() {
  ++it_;
  if(it_ != end_)
    pointee_.SetVisibileMapPoint(*it_);
  return *this;
}


}
}
}
}