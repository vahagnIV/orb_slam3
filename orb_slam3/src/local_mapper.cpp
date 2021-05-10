//
// Created by vahagn on 16/03/2021.
//

#include "local_mapper.h"
#include <frame/visible_map_point.h>
#include <logging.h>
#include <optimization/bundle_adjustment.h>

namespace orb_slam3 {

LocalMapper::LocalMapper(map::Atlas * atlas) : PositionObserver(), atlas_(atlas), thread_(nullptr) {}

LocalMapper::~LocalMapper() {
  Stop();
}

void LocalMapper::Run() {
  while (!cancelled_) {
    UpdateMessage message;
    GetUpdateQueue().wait_dequeue(message);
    switch (message.type) {
      case PositionMessageType::Final:continue;
      case PositionMessageType::Initial:continue;
      case PositionMessageType::Update: {
        std::cout << message.frame->Id() << std::endl;
        CreateNewMapPoints(message.frame);
      }
    }
  }
}

void LocalMapper::Start() {
  if (thread_ != nullptr)
    return;
  cancelled_ = false;
  thread_ = new std::thread(&LocalMapper::Run, this);
}

void LocalMapper::Stop() {
  if (nullptr == thread_)
    return;
  cancelled_ = true;
  thread_->join();
  delete thread_;
  thread_ = nullptr;
}

void LocalMapper::MapPointCulling(unordered_set<map::MapPoint *> & map_points) {
  unsigned erased = 0;
  for (auto mp: map_points)
    if (mp->GetVisible() / mp->GetFound() < 0.25) {
      EraseMapPoint(mp);
      ++erased;
    }
  logging::RetrieveLogger()->debug("LM: erased {} mps", erased);
}

void LocalMapper::EraseMapPoint(map::MapPoint * map_point) {
  for (auto obs: map_point->Observations()) {
    obs.first->EraseMapPoint(map_point);
    delete obs.second;
  }
  delete map_point;
}

void LocalMapper::ProcessNewKeyFrame(frame::FrameBase * frame) {
  std::unique_ptr<std::unordered_set<frame::Observation *>> observations = frame->GetRecentObservations();
  for (auto observation: *observations) {
    frame->AddMapPoint(observation);
    recently_added_map_points_.insert(observation->GetMapPoint());
  }
  frame->CovisibilityGraph().Update();
}

bool LocalMapper::CreateNewMapPoints(frame::FrameBase * frame) {
  //TODO: decide the number of covisible frames from the type, i.e. Monocular 20,  else 10
  auto covisible_frames = frame->CovisibilityGraph().GetCovisibleKeyFrames(20);
  std::unordered_set<map::MapPoint *> new_map_points;
  for (auto existing_keyframe: covisible_frames) {
    frame->CreateNewMapPoints(existing_keyframe);
    auto observations = frame->GetRecentObservations();
    std::transform(observations->begin(),
                   observations->end(),
                   std::inserter(new_map_points, new_map_points.begin()),
                   [](frame::Observation * observation) { return observation->GetMapPoint(); });

  }

  // TODO: make bundle adjustment

  g2o::SparseOptimizer optimizaer;
  optimization::InitializeOptimizer(optimizaer);
  optimization::BundleAdjustment(optimizaer, {}, new_map_points, 5);
  return true;

}

void LocalMapper::Optimize(frame::FrameBase * frame) {

}

void LocalMapper::AddKeyFrame(frame::FrameBase * frame) {
  ProcessNewKeyFrame(frame);
  MapPointCulling(recently_added_map_points_);
  CreateNewMapPoints(frame);
  Optimize(frame);

}

}