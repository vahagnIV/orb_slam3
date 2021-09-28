//
// Created by vahagn on 11/05/2021.
//

#include "observation.h"
#include <optimization/edges/se3_project_xyz_pose.h>
#include "monocular/monocular_key_frame.h"

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
    auto edge = new optimization::edges::SE3ProjectXYZPose(monocular_key_frame->GetCamera(),
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



/*
Observation & Observation::operator=(const Observation & other) {
  map_point_ = other.map_point_;
  key_frame_ = other.key_frame_;
//  feature_ids_ = other.feature_ids_;
  return *this;
}*/

}
}
