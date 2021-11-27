//
// Created by vahagn on 12/8/20.
//

#include <cassert>

#include "tracker.h"
#include <settings.h>
#include <messages/messages.h>
#include "constants.h"
#include "logging.h"
#include <frame/key_frame.h>
#include <optimization/bundle_adjustment.h>
#include <map/map_point.h>
#include <serialization/serialization_context.h>
#include <factories/frame_factory.h>

#include "local_mapper.h"
#include "debug/debug_utils.h"

namespace orb_slam3 {

const size_t MAX_FRAMES = 30;
const size_t MIN_FRAMES = 0;
const size_t MIN_ACCEPTABLE_TRACKED_MAP_POINTS_COUNT = 15;

Tracker::Tracker(orb_slam3::map::Atlas * atlas, LocalMapper * local_mapper)
    : atlas_(atlas),
      velocity_is_valid_(false),
      last_frame_(nullptr),
      state_(NOT_INITIALIZED),
      local_mapper_(local_mapper) {
  kf_counter_ = 1000;
}

Tracker::~Tracker() {
//  delete atlas_;
}

map::Atlas * Tracker::GetAtlas() const {
  return atlas_;
}

Tracker::State Tracker::GetState() const {
  return state_;
}

frame::KeyFrame * Tracker::ListLocalKeyFrames(frame::Frame * current_frame,
                                              std::unordered_set<frame::KeyFrame *> & out_local_keyframes) {
  out_local_keyframes.clear();
  std::unordered_set<map::MapPoint *> frame_map_points;
  current_frame->ListMapPoints(frame_map_points);
  std::unordered_map<frame::KeyFrame *, unsigned> key_frame_counter;
  for (auto & map_point: frame_map_points) {
    if(map_point->IsBad())
      continue;
    for (const auto & observation: map_point->Observations()) ++key_frame_counter[observation.second.GetKeyFrame()];
  }

  frame::KeyFrame * max_covisible_key_frame = nullptr;
  unsigned max_count = 0;
  for (auto & kf_count: key_frame_counter) {
    out_local_keyframes.insert(kf_count.first);
    if (max_count < kf_count.second) {
      max_count = kf_count.second;
      max_covisible_key_frame = kf_count.first;
    }
  }

  for (auto kf: key_frame_counter) {
    auto covisible_frames = kf.first->GetCovisibilityGraph().GetCovisibleKeyFrames(10);
    if (out_local_keyframes.size() >= 80)
      break;
    std::copy(covisible_frames.begin(),
              covisible_frames.end(),
              std::inserter(out_local_keyframes, out_local_keyframes.begin()));
  }

  return max_covisible_key_frame;
}

bool Tracker::TrackWithMotionModel(frame::Frame * frame, std::list<frame::MapPointVisibilityParams> & out_visibles) {
  if (!velocity_is_valid_)
    return false;
//  std::unordered_set<map::MapPoint *> all_candidate_map_points;
  PredictAndSetNewFramePosition(frame);
  return frame->EstimatePositionByProjectingMapPoints(last_frame_, out_visibles);

}

bool Tracker::TrackWithReferenceKeyFrame(frame::Frame * frame) {
  frame->SetStagingPosition(reference_keyframe_->GetPosition());
  frame->ApplyStaging();
  return frame->FindMapPointsFromReferenceKeyFrame(reference_keyframe_);
}

void Tracker::StartNewMap(frame::Frame * frame) {
  atlas_->CreateNewMap();
  frame->SetIdentity();
  state_ = FIRST_IMAGE;
  frame->SetMap(atlas_->GetCurrentMap());
  ReplaceLastFrame(frame);
  this->last_key_frame_ = nullptr;
  this->reference_keyframe_ = nullptr;
  this->velocity_is_valid_ = false;
}

TrackingResult Tracker::TrackInOkState(frame::Frame * frame) {

  last_frame_->UpdateFromReferenceKeyFrame();

  assert(OK == state_);
  ++kf_counter_;

  std::list<frame::MapPointVisibilityParams> frame_visibles;
  bool tracked = (TrackWithMotionModel(frame, frame_visibles) || TrackWithReferenceKeyFrame(frame));
//  bool tracked =TrackWithReferenceKeyFrame(frame);

  if (!tracked) {
    // TODO: go to relocalization
    velocity_is_valid_ = false;
    delete frame;
    state_ = LOST;
    return TrackingResult::TRACKING_FAILED;
  }

//  debug::DrawCommonMapPoints(frame->GetFilename(),
//                             reference_keyframe_->GetFilename(),
//                             dynamic_cast<frame::monocular::MonocularFrame *>(frame),
//                             dynamic_cast<frame::monocular::MonocularKeyFrame *>(reference_keyframe_));

  std::unordered_set<frame::KeyFrame *> local_keyframes;
  frame::KeyFrame * reference_keyframe = ListLocalKeyFrames(frame, local_keyframes);
  if (reference_keyframe)
    reference_keyframe_ = reference_keyframe;

  frame::Frame::MapPointSet current_frame_map_points, local_map_points_except_current;
  frame->ListMapPoints(current_frame_map_points);
  for (auto mp: current_frame_map_points)
    mp->IncreaseVisible();

  for (auto keyframe: local_keyframes) {
    frame::Frame::MapPointSet key_frame_map_points;
    keyframe->ListMapPoints(key_frame_map_points);
    for (auto mp: key_frame_map_points) {
      if (current_frame_map_points.find(mp) == current_frame_map_points.end())
        local_map_points_except_current.insert(mp);
    }
  }

  std::list<frame::MapPointVisibilityParams> visible_map_points;
  frame->FilterVisibleMapPoints(local_map_points_except_current,
                                visible_map_points,
                                frame->GetSensorConstants()->projection_search_radius_multiplier);
  for (auto & vmp: visible_map_points)
    vmp.map_point->IncreaseVisible();

  logging::RetrieveLogger()->debug("Filtering visible map points: {} / {}",
                                   visible_map_points.size(),
                                   local_map_points_except_current.size());

  frame->SearchInVisiblePoints(visible_map_points);

  frame->OptimizePose();
  if (frame->GetMapPointsCount() < 20) {
    return TrackingResult::TRACKING_FAILED;
  }
  frame->SetMap(atlas_->GetCurrentMap());
  ComputeVelocity(frame, last_frame_);

  if (Settings::Get().MessageRequested(messages::MessageType::TRACKING_INFO)) {
    messages::MessageProcessor::Instance().Enqueue(new messages::TrackingInfo(frame->GetPosition(),
                                                                              frame->GetTimeCreated(),
                                                                              velocity_));
  }

  debug::DisplayTrackingInfo(frame,
                             last_frame_,
                             atlas_->GetCurrentMap(),
                             frame_visibles,
                             visible_map_points,
                             local_map_points_except_current);

  frame::Frame::MapPointSet map_points;
  frame->ListMapPoints(map_points);
  for (auto map_point: map_points)
    map_point->IncreaseFound();

  //TODO: Add keyframe if necessary
  if (NeedNewKeyFrame(frame)) {
    auto keyframe = frame->CreateKeyFrame();
    last_key_frame_ = keyframe;
    local_mapper_->AddToQueue(keyframe);
  }
#ifndef MULTITHREADED
  local_mapper_->RunIteration();
#endif
//  ComputeVelocity(frame, last_frame_);
  ReplaceLastFrame(frame);
  return TrackingResult::OK;
}

void Tracker::ReplaceLastFrame(frame::Frame * frame) {
  auto tmp = last_frame_;
  last_frame_ = frame;
  delete tmp;
}

bool Tracker::NeedNewKeyFrame(frame::Frame * frame) {
  bool more_than_max_frames_passed = frame->Id() >= last_relocalization_frame_id_ + MAX_FRAMES;
  size_t number_of_key_frames = atlas_->GetCurrentMap()->GetSize();
  if (/*local_mapper_.IsStopped() ||*/ (!more_than_max_frames_passed && number_of_key_frames > MAX_FRAMES)) {
    std::cout << "NeedNewKeyFrame = false0" << std::endl;
    return false;
  }
  bool local_mapper_accepts_key_frame = local_mapper_->AcceptKeyFrames();
  float th_ref_ratio = 0.9f; // for monocular only
  bool more_than_min_frames_passed = frame->Id() >= last_key_frame_->Id() + MIN_FRAMES;
  std::unordered_set<map::MapPoint *> map_points;
  frame->ListMapPoints(map_points);
  size_t tracked_points_count = map_points.size();
  size_t min_observation_count = number_of_key_frames < 3 ? 2 : 3;

  std::unordered_set<map::MapPoint *> ref_kf_all_tracked_map_points;
  reference_keyframe_->ListMapPoints(ref_kf_all_tracked_map_points);
  std::unordered_set<map::MapPoint *> filtered_tracked_map_points;
  for (auto tracked_map_point: ref_kf_all_tracked_map_points) {
    if (tracked_map_point->GetObservationCount() >= min_observation_count) {
      filtered_tracked_map_points.insert(tracked_map_point);
    }
  }
  bool few_tracked_points =
      tracked_points_count < filtered_tracked_map_points.size() * th_ref_ratio
          && tracked_points_count > MIN_ACCEPTABLE_TRACKED_MAP_POINTS_COUNT;
  if (!few_tracked_points) {
//    std::cout << "Tracked map points count: " << tracked_points_count << std::endl;
//    std::cout << "filtered_tracked_map_points.size(): " << filtered_tracked_map_points.size() << std::endl;
//    std::cout << "th_ref_ratio: " << th_ref_ratio << std::endl;
//    std::cout << "MIN_ACCEPTABLE_TRACKED_MAP_POINTS_COUNT: " << MIN_ACCEPTABLE_TRACKED_MAP_POINTS_COUNT << std::endl;
//    std::cout << "NeedNewKeyfrane !few_tracked points" << std::endl;
    return false;
  }
  if (!more_than_max_frames_passed && !more_than_min_frames_passed) {
//    std::cout << "NeedNewKeyFrame = false1" << std::endl;
    return false;
  }
//  std::cout << "NeedNewKeyFrame = " << local_mapper_accepts_key_frame << std::endl;
  return local_mapper_accepts_key_frame;
  /*std::unordered_set<map::MapPoint *> m;
  frame->ListMapPoints(m);
  bool need = kf_counter_ >= 4;// && m.size() > 40;
  if (need)
    kf_counter_ = 0;

  return need;*/
}

TrackingResult Tracker::TrackInFirstImageState(frame::Frame * frame) {

  assert(FIRST_IMAGE == state_);
  assert(frame->IsValid());
  frame->SetMap(atlas_->GetCurrentMap());
  if (frame->Link(last_frame_)) {
    map::Map * current_map = atlas_->GetCurrentMap();
    std::cout << "Position after linking " << std::endl;
    frame->GetPosition().print();

    frame::KeyFrame * initial_key_frame = last_frame_->CreateKeyFrame();
    initial_key_frame->SetInitial(true);

    frame::KeyFrame * current_key_frame = frame->CreateKeyFrame();
    last_key_frame_ = current_key_frame;

    initial_key_frame->Initialize();
    current_key_frame->Initialize();

    current_map->SetInitialKeyFrame(initial_key_frame);

    std::unordered_set<frame::KeyFrame *> key_frames{initial_key_frame, current_key_frame};
    std::unordered_set<map::MapPoint *> map_points;
    current_key_frame->ListMapPoints(map_points);

    optimization::BundleAdjustment(key_frames, map_points, 30);
    std::cout << "Position after linking BA " << std::endl;
    current_key_frame->GetPosition().print();

    std::vector<precision_t> depths;
    for (auto mp: map_points) depths.push_back(mp->GetPosition().z());

    auto pose = current_key_frame->GetPosition();
    pose.T /= depths[depths.size() / 2];
    current_key_frame->SetStagingPosition(pose);
    current_key_frame->ApplyStaging();
    frame->SetStagingPosition(pose);
    frame->ApplyStaging();

    std::sort(depths.begin(), depths.end());
    for (auto mp: map_points) {
      TPoint3D pose = mp->GetPosition();
      pose /= depths[depths.size() / 2];
      mp->SetStagingMinInvarianceDistance(mp->GetMinInvarianceDistance() / depths[depths.size() / 2]);
      mp->SetStagingMaxInvarianceDistance(mp->GetMaxInvarianceDistance() / depths[depths.size() / 2]);
      mp->SetStagingPosition(pose);
      mp->ComputeDistinctiveDescriptor();
      mp->CalculateNormalStaging();
      mp->ApplyStaging();
    }

    reference_keyframe_ = current_key_frame;
    state_ = OK;
    last_frame_ = frame;
    local_mapper_->AddToQueue(initial_key_frame);
    local_mapper_->AddToQueue(current_key_frame);
//    this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Initial, .frame=initial_key_frame});
//    this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Update, .frame=current_key_frame});


    // TODO: remove the following line in multithreading
#ifndef MULTITHREADED
    local_mapper_->RunIteration();
#endif

  } else {
    if (frame->Id() - last_frame_->Id() >= 40)
      StartNewMap(frame);
    else
      delete frame;

    return TrackingResult::LINKING_FAILED;
  }

  return TrackingResult::OK;
}

TrackingResult Tracker::TrackInNotInitializedState(frame::Frame * frame) {

  assert(NOT_INITIALIZED == state_);
  assert(frame->IsValid());
  frame->SetIdentity();
  frame->SetMap(atlas_->GetCurrentMap());

//  frame->SetInitial(true);
  state_ = FIRST_IMAGE;
  last_frame_ = frame;
  return TrackingResult::NEED_MORE_IMAGES;
}

TrackingResult Tracker::Track(frame::Frame * frame) {

  if (!frame->IsValid()) {
    return TrackingResult::INVALID_FRAME;
  }

  TrackingResult res = TrackingResult::OK;
  switch (state_) {
    case NOT_INITIALIZED: {
      res = TrackInNotInitializedState(frame);
      break;
    }
    case FIRST_IMAGE: {
      res = TrackInFirstImageState(frame);
      break;
    }
    case OK: {
      res = TrackInOkState(frame);
      break;
    }
    case LOST: {
      StartNewMap(frame);
      state_ = FIRST_IMAGE;
      res = TrackingResult::OK;
      break;
    }
    default: {
    }
  }
  return res;
}

void Tracker::ComputeVelocity(const geometry::RigidObject * object2, const geometry::RigidObject * object1) {
  velocity_ = object2->GetPositionWithLock() * object1->GetInversePosition();
  velocity_is_valid_ = true;
}

void Tracker::PredictAndSetNewFramePosition(frame::Frame * frame) const {
  geometry::Pose pose = velocity_ * last_frame_->GetPosition();
  frame->SetStagingPosition(pose);
  frame->ApplyStaging();
}

void Tracker::SaveState(std::ostream & ostream) {
  WRITE_TO_STREAM(kf_counter_, ostream);
  WRITE_TO_STREAM(velocity_is_valid_, ostream);
  ostream << velocity_;
  size_t last_kf_frame_id = last_key_frame_->Id();
  size_t reference_kf_frame_id = reference_keyframe_->Id();
  WRITE_TO_STREAM(last_kf_frame_id, ostream);
  WRITE_TO_STREAM(reference_kf_frame_id, ostream);
  WRITE_TO_STREAM(state_, ostream);
  size_t current_map_id = reinterpret_cast<size_t>(atlas_->GetCurrentMap());
  WRITE_TO_STREAM(current_map_id, ostream);

  frame::FrameType type = last_frame_->Type();
  WRITE_TO_STREAM(type, ostream);

  last_frame_->Serialize(ostream);
}

void Tracker::LoadState(std::istream & istream, serialization::SerializationContext & context) {
  READ_FROM_STREAM(kf_counter_, istream);
  READ_FROM_STREAM(velocity_is_valid_, istream);
  istream >> velocity_;

  size_t last_kf_frame_id, reference_kf_frame_id;
  READ_FROM_STREAM(last_kf_frame_id, istream);
  READ_FROM_STREAM(reference_kf_frame_id, istream);
  last_key_frame_ = context.kf_id[last_kf_frame_id];
  reference_keyframe_ = context.kf_id[reference_kf_frame_id];
  READ_FROM_STREAM(state_, istream);
  size_t current_map_id;
  READ_FROM_STREAM(current_map_id, istream);

  atlas_->SetCurrentMap(context.map_id[current_map_id]);

  frame::FrameType type;
  READ_FROM_STREAM(type, istream);

  last_frame_ = factories::FrameFactory::Create(type, istream, context);
  frame::Frame::next_id_ = last_frame_->Id() + 1;

}

}  // namespace orb_slam3
