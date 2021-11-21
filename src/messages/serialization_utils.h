//
// Created by vahagn on 12/11/2021.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_SERIALIZATION_UTILS_H_
#define ORB_SLAM3_SRC_MESSAGES_SERIALIZATION_UTILS_H_

#include <geometry/pose.h>

namespace orb_slam3 {
namespace messages {

#define COPY_TO(dest, value)  memcpy(dest, &value, sizeof(value)); \
                                dest+= sizeof(value);

#define COPY_FROM(source, value) memcpy((void *)&value, source, sizeof(value)); \
                                source+= sizeof(value);

#define DESERIALIZE_TYPE(source) MessageType type; \
                                COPY_FROM(source, type); \
                                assert(type == Type());

#define INIT_SERIALIZATION(serialized, size) MessageType type = Type(); \
                                       serialized.resize(size + sizeof(MessageType));   \
                                       uint8_t * dest = serialized.data(); \
                                       COPY_TO(dest, type);

#define INIT_DESERIALIZATION(serialized) const uint8_t * source = serialized.data(); \
                                         DESERIALIZE_TYPE(source);

#define POINT_SIZE (sizeof(decltype(geometry::Pose::T)::Scalar) * 3)
#define ROTATION_MATRIX_SIZE (sizeof(decltype(geometry::Pose::R)::Scalar) * 4)
#define POSITION_SIZE (ROTATION_MATRIX_SIZE + POINT_SIZE)

void SerializePose(const geometry::Pose & pose, uint8_t *& out_serialized);
void SerializePoint(const TPoint3D & point, uint8_t *& out_serialized);
void SerializeRotationMatrix(const TMatrix33 & R, uint8_t *& dest);

void DeSerializePose(const uint8_t *& source, geometry::Pose & pose);
void DeSerializePoint(const uint8_t *& source, TPoint3D & point);
void DeSerializeRotationMatrix(const uint8_t *& source, TMatrix33 & R);

}
}
#endif //ORB_SLAM3_SRC_MESSAGES_SERIALIZATION_UTILS_H_
