//
// Created by vahagn on 12/19/20.
//

#ifndef ORB_SLAM3_INCLUDE_ORB_VOCABULARY_H_
#define ORB_SLAM3_INCLUDE_ORB_VOCABULARY_H_
#include <DBoW2/DBoW2.h>

namespace orb_slam3{

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
    ORBVocabulary;

}
#endif //ORB_SLAM3_INCLUDE_ORB_VOCABULARY_H_
