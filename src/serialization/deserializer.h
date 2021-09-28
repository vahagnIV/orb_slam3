//
// Created by vahagn on 28.09.21.
//

#ifndef ORB_SLAM3_SRC_SERIALIZATION_DESERIALIZER_H_
#define ORB_SLAM3_SRC_SERIALIZATION_DESERIALIZER_H_

#include <map/atlas.h>
#include <typedefs.h>
#include "serialization_context.h"

namespace orb_slam3 {

namespace frame {
class Observation;
}

namespace serialization {

class Deserializer {
 public:
  map::Atlas *Deserialize(std::istream &istream, SerializationContext &context);

};

}
}
#endif //ORB_SLAM3_SRC_SERIALIZATION_DESERIALIZER_H_
