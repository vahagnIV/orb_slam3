//
// Created by vahagn on 11/05/2021.
//

#include "observation.h"
#include <optimization/edges/se3_project_xyz_pose.h>
#include "monocular/monocular_key_frame.h"

namespace orb_slam3 {
namespace frame {

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

g2o::BaseEdge<2, Eigen::Vector2d> * Observation::CreateBinaryEdge() {
  if (IsMonocular()) {
    const auto monocular_key_frame = dynamic_cast<const monocular::MonocularKeyFrame *>(key_frame_);
    auto edge = new optimization::edges::SE3ProjectXYZPose(monocular_key_frame->GetCamera());
    auto measurement = monocular_key_frame->GetFeatures().unprojected_keypoints[feature_ids_[0]];
    edge->setMeasurement(Eigen::Map<Eigen::Matrix<double,
                                                  2,
                                                  1>>(measurement.data()));
    precision_t
        information_coefficient =
        monocular_key_frame->GetFeatureExtractor()->GetAcceptableSquareError(monocular_key_frame->GetFeatures().keypoints[feature_ids_[0]].level);
    edge->setInformation(Eigen::Matrix2d::Identity() * information_coefficient);
    return edge;
  }
  throw std::runtime_error("Stereo is not yet implemented");
  return nullptr;
}

g2o::BaseEdge<2, Eigen::Vector2d> * Observation::CreateEdge() {
  throw std::runtime_error("Unary edges is not yet implemented");
  return nullptr;
}

void Observation::AppendDescriptorsToList(vector<features::DescriptorType> & out_descriptor_ptr) const {
  if (IsMonocular()) {
    const auto monocular_key_frame = dynamic_cast<const monocular::MonocularKeyFrame *>(key_frame_);
    out_descriptor_ptr.push_back(monocular_key_frame->GetFeatures().descriptors.row(feature_ids_[0]));
  } else
    throw std::runtime_error("Stereo is not yet implemented");

}

bool Observation::IsMonocular() const {
  return feature_ids_.size() == 1;
}

g2o::RobustKernel * Observation::CreateRobustKernel() {
  auto rk = new g2o::RobustKernelHuber;
  rk->setDelta(constants::HUBER_MONO_DELTA
                   * dynamic_cast<frame::monocular::MonocularKeyFrame *>(GetKeyFrame())->GetCamera()->FxInv());
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