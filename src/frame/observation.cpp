//
// Created by vahagn on 11/05/2021.
//

#include "observation.h"
#include <optimization/edges/se3_project_xyz_pose.h>
#include "monocular/monocular_key_frame.h"
#include <serialization/serialization_context.h>
#include <map/map_point.h>

namespace orb_slam3 {
namespace frame {

Observation::Observation() : map_point_(nullptr), key_frame_(nullptr), feature_ids_() {

}

Observation::Observation(map::MapPoint * map_point, KeyFrame * key_frame, size_t feature_id) : map_point_(map_point),
                                                                                               key_frame_(key_frame),
                                                                                               feature_ids_({feature_id}) {
}

Observation::Observation(map::MapPoint * map_point,
                         KeyFrame * key_frame,
                         size_t feature_id_left,
                         size_t feature_id_right) : map_point_(map_point),
                                                    key_frame_(key_frame),
                                                    feature_ids_({feature_id_left, feature_id_right}) {
}

Observation::Observation(const Observation & other)
    : map_point_(other.map_point_), key_frame_(other.key_frame_), feature_ids_(other.feature_ids_) {

}

Observation & Observation::operator=(const Observation & other) {
  key_frame_ = other.key_frame_;
  map_point_ = other.map_point_;
  feature_ids_ = other.feature_ids_;
  return *this;
}

optimization::edges::BABinaryEdge * Observation::CreateBinaryEdge() const {
  if (IsMonocular()) {
    const auto monocular_key_frame = dynamic_cast<const monocular::MonocularKeyFrame *>(key_frame_);
    auto edge = new optimization::edges::SE3ProjectXYZPose(monocular_key_frame->GetMonoCamera(),
                                                           constants::MONO_CHI2);
    const size_t & feature_id = feature_ids_[0];
    auto & kp = monocular_key_frame->GetFeatureHandler()->GetFeatures().keypoints[feature_id];
    edge->setMeasurement(kp.pt);
    precision_t
        information_coefficient =
        1. / monocular_key_frame->GetFeatureExtractor()->GetAcceptableSquareError(kp.level);
    edge->setInformation(Eigen::Matrix2d::Identity() * information_coefficient);
    return edge;
  }
  throw std::runtime_error("Stereo is not yet implemented");
  return nullptr;
}

optimization::edges::BAUnaryEdge * Observation::CreateEdge() const {
  throw std::runtime_error("Unary edges is not yet implemented");
  return nullptr;
}

void Observation::AppendDescriptorsToList(std::vector<features::DescriptorType> & out_descriptor_ptr) const {
  if (IsMonocular()) {
    const auto monocular_key_frame = dynamic_cast<const monocular::MonocularKeyFrame *>(key_frame_);
    out_descriptor_ptr.push_back(monocular_key_frame->GetFeatureHandler()->GetFeatures().descriptors.row(feature_ids_[0]));
  } else
    throw std::runtime_error("Stereo is not yet implemented");

}

size_t Observation::GetFeatureId() const {
  assert(IsMonocular());
  return feature_ids_[0];
}

size_t Observation::GetLeftFeatureId() const {
  assert(!IsMonocular());
  return feature_ids_[1];
}

size_t Observation::GetRightFeatureId() const {
  assert(!IsMonocular());
  return feature_ids_[0];
}

const std::vector<std::size_t>
Observation::GetFeatureIds() const {
  return feature_ids_;
}

bool Observation::IsMonocular() const {
  return feature_ids_.size() == 1;
}

g2o::RobustKernel * Observation::CreateRobustKernel() {
  auto rk = new g2o::RobustKernelHuber;
  rk->setDelta(constants::HUBER_MONO_DELTA);
  return rk;
}

void Observation::Serialize(std::ostream & ostream) const {
  assert(!map_point_->IsBad());

  size_t mp_id = reinterpret_cast<size_t>(map_point_);
  WRITE_TO_STREAM(mp_id, ostream);
  size_t kf_id = key_frame_->Id();
  WRITE_TO_STREAM(kf_id, ostream);
  size_t feature_count = feature_ids_.size();
  WRITE_TO_STREAM(feature_count, ostream);
  ostream.write((char *) feature_ids_.data(), sizeof(decltype(feature_ids_)::value_type) * feature_count);
  if (feature_ids_[0] == 0)
    std::cout << "Mp id: " << mp_id << " " << " Kf id: " << kf_id << std::endl;
}

void Observation::Deserialize(std::istream & istream, serialization::SerializationContext & context) {
  size_t mp_id;
  READ_FROM_STREAM(mp_id, istream);
  map::MapPoint * mp = context.mp_id[mp_id];
  assert(nullptr != mp);

  size_t kf_id;
  READ_FROM_STREAM(kf_id, istream);
  frame::KeyFrame * kf = context.kf_id[kf_id];
  assert(nullptr != kf);

  size_t feature_count;
  READ_FROM_STREAM(feature_count, istream);
  feature_ids_.resize(feature_count);
  istream.read((char *) feature_ids_.data(), sizeof(decltype(feature_ids_)::value_type) * feature_count);

  map_point_ = mp;
  key_frame_ = kf;
  key_frame_->AddMapPoint(*this);
}

}
}
