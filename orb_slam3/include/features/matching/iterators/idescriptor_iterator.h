//
// Created by vahagn on 24/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_I_DESCRIPTOR_ITERATOR_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_I_DESCRIPTOR_ITERATOR_H_
// === stl ===
#include <cstddef>

namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {

template<typename IteratorType>
class IJointDescriptorIterator {
 public:
  virtual IJointDescriptorIterator &operator++() = 0;
  virtual size_t IdxTo() = 0;
  virtual IteratorType begin() = 0;
  virtual IteratorType end() = 0;
  virtual bool IsValid() = 0;
  virtual ~IJointDescriptorIterator() = default;
};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_I_DESCRIPTOR_ITERATOR_H_
