//
// Created by vahagn on 11/10/2021.
//

#include "dbo_w2_vocabulary.h"
#include "constants.h"

namespace orb_slam3 {
namespace features {
namespace utils {

DBoW2Vocabulary::DBoW2Vocabulary() : vocabulary_(nullptr) {
}

DBoW2Vocabulary & DBoW2Vocabulary::Instance() {

  static DBoW2Vocabulary instance;
  if (nullptr == instance.vocabulary_) {
    const char * val = std::getenv(constants::BOW_VOCABULARY_FILE_PATH.c_str());
    if (nullptr == val) {
      std::stringstream ss;
      ss << "Could not find the environment variable " << constants::BOW_VOCABULARY_FILE_PATH;
      throw std::runtime_error(ss.str());
    }
    std::string bow_vocabulary_file_path(val);
    std::cout << "Loading Bow vocabulary from " << bow_vocabulary_file_path << std::endl;
    instance.vocabulary_ = new BowVocabulary;
    instance.vocabulary_->loadFromTextFile(bow_vocabulary_file_path);
    std::cout << "Done" << std::endl;
  }

  return instance;
}

const BowVocabulary * DBoW2Vocabulary::GetVocabulary() {
  return vocabulary_;
}

DBoW2Vocabulary::~DBoW2Vocabulary() {
  Destroy();
}

void DBoW2Vocabulary::Destroy() {
  delete Instance().vocabulary_;
  Instance().vocabulary_ = nullptr;
}

}
}
}