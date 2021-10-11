//
// Created by vahagn on 11/10/2021.
//

#include <features/bow/dbo_w2_vocabulary.h>
#include "keyframe_database_factory.h"
#include "frame/database/DBoW2/dbo_w2_database.h"

namespace orb_slam3 {
namespace factories {

frame::IKeyFrameDatabase * KeyframeDatabaseFactory::CreateKeyFrameDatabase(frame::KeyframeDatabaseType type,
                                                                           std::istream & istream,
                                                                           serialization::SerializationContext & contex) {
  switch (type) {
    case frame::KeyframeDatabaseType::DBoW2DB: {
      return new frame::DBoW2Database(istream, contex);
    }
    default:
      return nullptr;
  }
}

frame::IKeyFrameDatabase * KeyframeDatabaseFactory::CreateKeyFrameDatabase(frame::KeyframeDatabaseType type) {
  switch (type) {
    case frame::KeyframeDatabaseType::DBoW2DB: {
      return new frame::DBoW2Database(features::utils::DBoW2Vocabulary::Instance().GetVocabulary());
    }
    default:
      return nullptr;
  }
}

}
}