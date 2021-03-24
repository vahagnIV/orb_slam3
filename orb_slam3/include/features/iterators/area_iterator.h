//
// Created by vahagn on 24/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_ITERATORS_AREA_ITERATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_ITERATORS_AREA_ITERATOR_H_

#include "idescriptor_iterator.h"
#include <features/features.h>

namespace orb_slam3 {
namespace features {
namespace iterators {

class AreaIterator : public IJointDescriptorIterator<std::vector<std::size_t>::iterator> {
 public:
  AreaIterator(const Features &features_from, const Features &features_to, size_t window_size);
  AreaIterator &operator++() override;
  size_t IdxTo() override;
  bool IsValid() override;
  vector<size_t>::iterator begin() override;
  vector<size_t>::iterator end() override;
 private:
  size_t to_idx_;
  std::vector<std::size_t> selected_features_;
  const Features * features_from_;
  const Features * features_to_;
  bool is_valid_;
  size_t window_size_;

};

}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_ITERATORS_AREA_ITERATOR_H_
