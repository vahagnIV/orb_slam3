//
// Created by vahagn on 11.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_BOW_VOCABULARY_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_BOW_VOCABULARY_H_

#include <DBoW2/FORB.h>
#include <DBoW2/TemplatedVocabulary.h>

namespace orb_slam3 {
namespace features {
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> BowVocabulary;
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FEATURES_BOW_VOCABULARY_H_
