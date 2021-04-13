//
// Created by vahagn on 12/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_AREA_FROM_POINTEE_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_AREA_FROM_POINTEE_H_
namespace orb_slam3 {
namespace features {
namespace matching {
namespace iterators {
class AreaFromIterator;
class AreaToIterator;

class AreaFromPointee {
  friend class AreaFromIterator;
  friend class AreaToIterator;
 public:
  typedef size_t IdType;
  AreaFromPointee() = default;
  AreaFromPointee(DescriptorSet *descriptors, IdType id) : id_(id), descriptors_(descriptors) {}
  const DescriptorType GetDescriptor() { return descriptors_->row(id_); }
  const IdType GetId() const { return id_; }
 protected:
  IdType id_;
  DescriptorSet *descriptors_;
};

}
}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_MATCHING_ITERATORS_AREA_FROM_POINTEE_H_
