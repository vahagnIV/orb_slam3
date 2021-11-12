//
// Created by vahagn on 12/11/2021.
//

#include "serialization_utils.h"

namespace orb_slam3 {
namespace messages {

void SerializePose(const geometry::Pose & pose, uint8_t *& serialized) {
  SerializeRotationMatrix(pose.R, serialized);
  SerializePoint(pose.T, serialized);
}

void SerializePoint(const TPoint3D & T, uint8_t *& dest) {
  memcpy(dest, T.data(), 3 * sizeof(std::remove_reference<decltype(T)>::type::Scalar));
  dest += 3 * sizeof(std::remove_reference<decltype(T)>::type::Scalar);
}

void SerializeRotationMatrix(const TMatrix33 & R, uint8_t *& dest) {
  typedef Eigen::Quaternion<std::remove_reference<decltype(R)>::type::Scalar> QType;
  QType quaternion(R);
  memcpy(dest, &quaternion.x(), sizeof(QType::Scalar));
  dest += sizeof(QType::Scalar);
  memcpy(dest, &quaternion.y(), sizeof(QType::Scalar));
  dest += sizeof(QType::Scalar);
  memcpy(dest, &quaternion.z(), sizeof(QType::Scalar));
  dest += sizeof(QType::Scalar);
  memcpy(dest, &quaternion.w(), sizeof(QType::Scalar));
  dest += sizeof(QType::Scalar);;
}

void DeSerializePose(const uint8_t *& source, geometry::Pose & pose) {
  DeSerializeRotationMatrix(source, pose.R);
  DeSerializePoint(source, pose.T);
}

void DeSerializePoint(const uint8_t *& source, TPoint3D & T) {
  typedef std::remove_reference<decltype(T)>::type::Scalar ScalarType;
  memcpy((void *) T.data(), source, 3 * sizeof(ScalarType));
  source += 3 * sizeof(ScalarType);
}

void DeSerializeRotationMatrix(const uint8_t *& source, TMatrix33 & R) {
  typedef std::remove_reference<decltype(R)>::type::Scalar ScalarType;
  ScalarType x, y, z, w;
  memcpy((void *) &x, source, sizeof(x));
  source += sizeof(x);
  memcpy((void *) &y, source, sizeof(y));
  source += sizeof(y);
  memcpy((void *) &z, source, sizeof(z));
  source += sizeof(z);
  memcpy((void *) &w, source, sizeof(w));
  source += sizeof(w);
  typedef Eigen::Quaternion<ScalarType> QType;
  QType quat(w, x, y, z);
  R = quat.toRotationMatrix();
}

}
}