//
// Created by vahagn on 17/05/2021.
//

#include "monocular_optimization.h"
#include "utils.h"
#include "vertices/frame_vertex.h"
#include "vertices/map_point_vertex.h"
#include "edges/se3_project_xyz_pose_only.h"
#include <logging.h>

namespace orb_slam3 {
namespace optimization {

using namespace frame::monocular;

void OptimizePose(MonocularFrame * frame) {
  BaseMonocular::MonocularMapPoints map_points = frame->GetMapPoints();

  static const precision_t delta_mono = constants::HUBER_MONO_DELTA * frame->GetCamera()->FxInv();
  static const precision_t
      chi2_threshold = constants::MONO_CHI2 * frame->GetCamera()->FxInv() * frame->GetCamera()->FxInv();

  g2o::SparseOptimizer optimizer;
  InitializeOptimizer(optimizer);

  geometry::Pose pose = frame->GetPosition();

  auto frame_vertex = new vertices::FrameVertex(frame);
  size_t max_id = frame->Id();
  frame_vertex->setId(max_id);
  optimizer.addVertex(frame_vertex);

  const auto & features = frame->GetFeatures();

  for (auto mp_id: map_points) {
    map::MapPoint * map_point = mp_id.second;
    size_t feature_id = mp_id.first;
    if (map_point->IsBad())
      continue;
    auto edge = new edges::SE3ProjectXYZPoseOnly(map_point, feature_id, frame->GetCamera(), map_point->GetPosition());
    edge->setId(++max_id);
    edge->setVertex(0, frame_vertex);
    edge->setMeasurement(Eigen::Map<const Eigen::Matrix<double,
                                                        2,
                                                        1>>(features.unprojected_keypoints[feature_id].data()));
    auto rk = new g2o::RobustKernelHuber;
    rk->setDelta(delta_mono);
    edge->setLevel(0);
    edge->setRobustKernel(rk);
    optimizer.addEdge(edge);
  }

  const int N = 4;

  size_t discarded_count = 0;
  for (int i = 0; i < N; ++i) {
    frame_vertex->setEstimate(pose.GetQuaternion());
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
    for (auto edge_base: optimizer.edges()) {
      auto edge = dynamic_cast<edges::SE3ProjectXYZPoseOnly *>(edge_base);
      if (1 == edge->level()) { // If  the edge was not included in the optimization
        edge->computeError();
      }
      if (edge->chi2() < chi2_threshold) {
        edge->setLevel(0);
      } else {
        if (N - 1 == i) {
          frame->EraseMapPoint(edge->GetFeatureId());
          ++discarded_count;
        } else
          edge->setLevel(1);
      }
      if (N - 2 == i)
        edge->setRobustKernel(nullptr);
    }
  }

  logging::RetrieveLogger()->debug("Pose optimization discarded {} map_points", discarded_count);

  frame->SetPosition(frame_vertex->estimate());

}

}
}