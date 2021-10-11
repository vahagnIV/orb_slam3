//
// Created by vahagn on 11/10/2021.
//

#ifndef ORB_SLAM3_SRC_FEATURES_UTILS_D_BO_W_2_VOCABULARY_H_
#define ORB_SLAM3_SRC_FEATURES_UTILS_D_BO_W_2_VOCABULARY_H_

#include <DBoW2/FORB.h>
#include <DBoW2/TemplatedVocabulary.h>
#include "bow_vocabulary.h"

namespace orb_slam3 {
namespace features {
namespace utils {

class DBoW2Vocabulary {
 public:
  ~DBoW2Vocabulary();
  static DBoW2Vocabulary & Instance();
  static void Destroy();
  const BowVocabulary * GetVocabulary();
 private:
  explicit DBoW2Vocabulary();
 private:
  BowVocabulary * vocabulary_;
  std::string path_;



};

}
}
}
#endif //ORB_SLAM3_SRC_FEATURES_UTILS_D_BO_W_2_VOCABULARY_H_
