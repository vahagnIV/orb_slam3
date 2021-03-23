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
  DBoW2::FeatureVector feature_vector;
  DBoW2::BowVector bow_vector;
  BowVocabulary *vocabulary;
  void ComputeBow(const std::vector<cv::Mat> &descriptors);
};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_BOW_CONTAINER_H_
