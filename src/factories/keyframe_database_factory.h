//
// Created by vahagn on 11/10/2021.
//

#ifndef ORB_SLAM3_SRC_FACTORIES_KEYFRAME_DATABASE_FACTORY_H_
#define ORB_SLAM3_SRC_FACTORIES_KEYFRAME_DATABASE_FACTORY_H_
#include <frame/database/ikey_frame_database.h>
namespace orb_slam3 {

namespace map {
class Atlas;
}

namespace serialization {
class SerializationContext;
}

namespace factories {
class KeyframeDatabaseFactory {
 public:
  static frame::IKeyFrameDatabase * CreateKeyFrameDatabase(frame::KeyframeDatabaseType type,
                                                           std::istream & istream,
                                                           serialization::SerializationContext & contex);
  static frame::IKeyFrameDatabase * CreateKeyFrameDatabase(frame::KeyframeDatabaseType type);
};

}
}
#endif //ORB_SLAM3_SRC_FACTORIES_KEYFRAME_DATABASE_FACTORY_H_
