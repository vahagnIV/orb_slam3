//
// Created by vahagn on 23/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_BOW_CONTAINER_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_BOW_CONTAINER_H_

// === dbow2 ===
#include <DBoW2/FeatureVector.h>
#include <DBoW2/BowVector.h>

// === orb_slam3 ===
#include <features/bow_vocabulary.h>

namespace orb_slam3 {
namespace features {

struct BowContainer {
  BowContainer();
  DBoW2::FeatureVector feature_vector;
  DBoW2::BowVector bow_vector;
  BowVocabulary *vocabulary;
  void ComputeBow(const std::vector<cv::Mat> &descriptors);
  class iterator {
    friend class BowContainer;
   public:
    iterator &operator++();
    iterator operator++(int);
    bool operator==(const iterator &other) const;
    bool operator!=(const iterator &other) const;
    const std::vector<unsigned> &ToIdx();
    const std::vector<unsigned> &FromIdx();
   private:
    iterator(const DBoW2::FeatureVector &first, const DBoW2::FeatureVector &second);
    void AdvanceEquilize();
    void SetToEnd();

   private:
    const DBoW2::FeatureVector *to_feature_vec_ptr_;
    const DBoW2::FeatureVector *from_feature_vec_ptr_;
    DBoW2::FeatureVector::const_iterator to_it;
    DBoW2::FeatureVector::const_iterator from_it;
  };
  iterator Begin(const BowContainer & other) const;
  iterator End() const;
 private:
  iterator end_iterator_;

};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_BOW_CONTAINER_H_
