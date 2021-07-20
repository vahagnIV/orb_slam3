//
// Created by vahagn on 12/05/2021.
//

#include "utils.h"

// === g2o ===

#include <g2o/solvers/dense/linear_solver_dense.h>
#include <map/map_point.h>
#include <frame/monocular/monocular_frame.h>

namespace orb_slam3 {
namespace optimization {

//void InitializeOptimizer(g2o::SparseOptimizer & optimizer) {
//  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>
//      linearSolver(new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>());
//
//  std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));
//
//  auto * solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
//  optimizer.setAlgorithm(solver);
//  optimizer.setVerbose(true);
//}

size_t FillKeyFrameVertices(const std::unordered_set<frame::KeyFrame *> & key_frames,
                            g2o::SparseOptimizer & inout_optimizer,
                            std::unordered_map<frame::KeyFrame *, vertices::FrameVertex *> & out_frame_map,
                            bool fixed) {
  size_t max_id = 0;
  for (auto key_frame: key_frames) {
    if (key_frame->IsBad())
      continue;
    auto vertex = new vertices::FrameVertex(key_frame);
    vertex->setId(key_frame->Id());
    max_id = std::max(max_id, static_cast<size_t>(vertex->id()));
    vertex->setFixed(fixed || key_frame->IsInitial());
    out_frame_map[key_frame] = vertex;
    inout_optimizer.addVertex(vertex);
  }
  return max_id;
}

void FillMpVertices(const std::unordered_set<map::MapPoint *> & map_points,
                    g2o::SparseOptimizer & inout_optimizer,
                    const std::unordered_map<frame::KeyFrame *, vertices::FrameVertex *> & frame_map,
                    std::unordered_map<map::MapPoint *, vertices::MapPointVertex *> & out_mp_map,
                    bool robust,
                    size_t & inout_id) {
  for (auto map_point: map_points) {
    if (map_point->IsBad())
      continue;

    auto mp_vertex = new vertices::MapPointVertex(map_point);

    mp_vertex->setId(++inout_id);
    inout_optimizer.addVertex(mp_vertex);
    unsigned number_of_edges = 0;
    for (auto observation: map_point->Observations()) {
      auto it = frame_map.find(observation.second.GetKeyFrame());
      if (it == frame_map.end())
        continue;

      auto * frame_vertex = dynamic_cast<vertices::FrameVertex *>(it->second);
      optimization::edges::BABinaryEdge * edge = observation.second.CreateBinaryEdge();
      edge->setVertex(0, frame_vertex);
      edge->setVertex(1, mp_vertex);
      edge->setId(++inout_id);
      if (robust)
        edge->setRobustKernel(observation.second.CreateRobustKernel());
      inout_optimizer.addEdge(edge);
      ++number_of_edges;
    }
    if (0 == number_of_edges)
      inout_optimizer.removeVertex(mp_vertex);
    else
      out_mp_map[map_point] = mp_vertex;
  }

}

g2o::VertexPointXYZ * CreateVertex(const map::MapPoint * map_point, const geometry::Pose & pose) {
  auto to_mp_vertex = new g2o::VertexPointXYZ();
  to_mp_vertex->setEstimate(pose.Transform(map_point->GetPosition()));
  return to_mp_vertex;
}

g2o::VertexSim3Expmap * CreateSim3Vertex(const geometry::Sim3Transformation & initial_guess,
                                         const camera::MonocularCamera * to_camera,
                                         const camera::MonocularCamera * from_camera) {
  g2o::Sim3 sim3(initial_guess.R, initial_guess.T, initial_guess.s);
  auto transformation_vertex = new g2o::VertexSim3Expmap();
  transformation_vertex->setEstimate(sim3);
  transformation_vertex->_focal_length1 << to_camera->Fx(), to_camera->Fy();
  transformation_vertex->_focal_length2 << from_camera->Fx(), from_camera->Fy();
  transformation_vertex->_principle_point1 << to_camera->Cx(), to_camera->Cy();
  transformation_vertex->_principle_point2 << from_camera->Cx(), from_camera->Cy();
  return transformation_vertex;
}

int Sim3FillOptimizer(g2o::SparseOptimizer & optimizer,
                      const frame::monocular::MonocularKeyFrame * const to_frame,
                      const frame::monocular::MonocularKeyFrame * const from_frame,
                      const geometry::Sim3Transformation & in_out_transformation,
                      const std::unordered_map<map::MapPoint *, size_t> & matches,
                      const std::unordered_map<map::MapPoint *, int> & predicted_levels) {

  const geometry::Pose & to_pose = to_frame->GetPosition();
  const geometry::Pose & from_pose = from_frame->GetPosition();
  const features::Features & to_features = to_frame->GetFeatureHandler()->GetFeatures();
  const features::Features & from_features = from_frame->GetFeatureHandler()->GetFeatures();

  int id_counter = 0;
  auto transformation_vertex = CreateSim3Vertex(in_out_transformation, to_frame->GetCamera(), from_frame->GetCamera());
  transformation_vertex->_fix_scale = false;
  transformation_vertex->setId(id_counter);

  optimizer.addVertex(transformation_vertex);

  auto to_map_points = to_frame->GetMapPoints();
  std::unordered_map<size_t, map::MapPoint *> inverted_matches;
  std::transform(matches.begin(),
                 matches.end(),
                 std::inserter(inverted_matches, inverted_matches.begin()),
                 [](const std::pair<map::MapPoint *, size_t> & pair) {
                   return std::pair<size_t, map::MapPoint *>(pair.second, pair.first);
                 });
  geometry::Sim3Transformation sim3_transformation_inverse = in_out_transformation.GetInversePose();

  for (auto it: to_map_points) {
    size_t feature_id = it.first;
    map::MapPoint * to_mp = it.second;
    assert(nullptr != to_mp);
    if (to_mp->IsBad())
      continue;

    auto match_it = inverted_matches.find(feature_id);
    if (match_it == inverted_matches.end()) continue;

    map::MapPoint * from_mp = match_it->second;
    assert(nullptr != from_mp);
    if (from_mp->IsBad()) continue;

    ++id_counter;

    // Create vertex for the map point in "from" coordinate frame
    auto from_mp_vertex = CreateVertex(from_mp, to_pose);
    from_mp_vertex->setId(4 * id_counter - 3);
    from_mp_vertex->setFixed(true);
    optimizer.addVertex(from_mp_vertex);

    auto from_to_edge = CreateEdge<g2o::EdgeSim3ProjectXYZ>(to_features,
                                                            to_frame->GetFeatureExtractor(),
                                                            to_features.undistorted_keypoints[feature_id],
                                                            to_features.keypoints[feature_id].level);
    from_to_edge->setId(4 * id_counter - 2);
    from_to_edge->setVertex(0, from_mp_vertex);
    from_to_edge->setVertex(1, optimizer.vertex(0));
    optimizer.addEdge(from_to_edge);

    // Create vertex for the map point in "to" coordinate frame
    auto to_mp_vertex = CreateVertex(to_mp, from_pose);
    to_mp_vertex->setId(4 * id_counter - 1);
    to_mp_vertex->setFixed(true);
    optimizer.addVertex(to_mp_vertex);

    int level;
    TPoint2D measurement;

    if (from_mp->IsInKeyFrame(from_frame)) {
      const auto & obs = from_mp->Observation(from_frame);
      measurement = from_features.undistorted_keypoints[obs.GetFeatureId()];
      level = from_features.keypoints[obs.GetFeatureId()].level;
      std::cout << "Mp is in keyframe" << std::endl;
    } else {
      auto level_it = predicted_levels.find(from_mp);
      assert(level_it != predicted_levels.end());
      level = level_it->second;
      TPoint3D virtual_point = sim3_transformation_inverse.Transform(from_pose.Transform(to_mp->GetPosition()));
      if (virtual_point.z() <= 0)
        continue;
      from_frame->GetCamera()->ProjectPoint(virtual_point, measurement);
    }

//    auto to_from_edge = CreateEdge<g2o::EdgeInverseSim3ProjectXYZ>(from_features,
//                                                                   from_frame->GetFeatureExtractor(),
//                                                                   measurement,
//                                                                   level);
    auto to_from_edge = CreateEdge<MyInverse>(from_features,
                                              from_frame->GetFeatureExtractor(),
                                              measurement,
                                              level);

    to_from_edge->setId(4 * id_counter);
    to_from_edge->setVertex(0, to_mp_vertex);
    to_from_edge->setVertex(1, optimizer.vertex(0));
    optimizer.addEdge(to_from_edge);
  }
  return id_counter;

}

}
}