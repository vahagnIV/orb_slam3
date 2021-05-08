//
// Created by vahagn on 08.05.21.
//

#include <g2o/core/robust_kernel_impl.h>

#include "frame/monocular_observation.h"
#include <optimization/edges/se3_project_xyz_pose.h>

namespace orb_slam3 {
namespace frame {

MonocularObservation::MonocularObservation(MonocularFrame * frame, size_t feature_id)
    : frame_(frame), feature_id_(feature_id_) {
}

g2o::BaseEdge<2, Eigen::Vector2d> * MonocularObservation::CreateMultiEdge() {
  const precision_t delta_mono = constants::HUBER_MONO_DELTA * frame_->GetCamera()->FxInv();
  auto edge = new optimization::edges::SE3ProjectXYZPose(frame_->GetCamera().get());
  auto measurement = frame_->GetFeatures().unprojected_keypoints[feature_id_];
  edge->setMeasurement(Eigen::Map<Eigen::Matrix<double,
                                                2,
                                                1>>(measurement.data()));
  precision_t
      information_coefficient =
      frame_->GetFeatureExtractor()->GetAcceptableSquareError(frame_->GetFeatures().keypoints[feature_id_].level);
  edge->setInformation(Eigen::Matrix2d::Identity() * information_coefficient);
  edge->setId(Id());
  auto rk = new g2o::RobustKernelHuber;
  rk->setDelta(delta_mono);
  edge->setRobustKernel(rk);
  return edge;
}

g2o::BaseEdge<2, Eigen::Vector2d> * MonocularObservation::CreateEdge() {
  return nullptr;
}

void MonocularObservation::AppendDescriptorsToList(vector<features::DescriptorType> & out_descriptor_ptr) const {
  out_descriptor_ptr.emplace_back(frame_->GetFeatures().descriptors.row(static_cast<int>(feature_id_)));
}

}
}