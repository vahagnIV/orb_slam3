//
// Created by vahagn on 12/8/20.
//

#include <cassert>

#include "tracker.h"
#include "constants.h"
#include "logging.h"
#include <frame/key_frame.h>
#include <optimization/bundle_adjustment.h>

#include "local_mapper.h"
#include "debug/debug_utils.h"

namespace orb_slam3 {

const size_t MAX_FRAMES = 30;
const size_t MIN_FRAMES = 0;
const size_t MIN_ACCEPTABLE_TRACKED_MAP_POINTS_COUNT = 15;

Tracker::Tracker(orb_slam3::map::Atlas * atlas)
    : atlas_(atlas),
      velocity_is_valid_(false),
      last_frame_(nullptr),
      state_(NOT_INITIALIZED){
  kf_counter = 1000;
}

Tracker::~Tracker() {
  delete atlas_;
}

frame::KeyFrame * Tracker::ListLocalKeyFrames(frame::Frame * current_frame,
                                              unordered_set<frame::KeyFrame *> & out_local_keyframes) {
  out_local_keyframes.clear();
  std::unordered_set<map::MapPoint *> frame_map_points;
  current_frame->ListMapPoints(frame_map_points);
  std::unordered_map<frame::KeyFrame *, unsigned> key_frame_counter;
  for (auto & map_point: frame_map_points) {
    for (const auto & observation: map_point->Observations()) ++key_frame_counter[observation.first];
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
  frame->SetPosition(reference_keyframe_->GetPosition());
  return frame->FindMapPointsFromReferenceKeyFrame(reference_keyframe_);
}

int time_to_wait = 0;
int mode = 7;

TrackingResult Tracker::TrackInOkState(frame::Frame * frame) {
  last_frame_->UpdateFromReferenceKeyFrame();

  assert(OK == state_);
  ++kf_counter;

  std::list<frame::MapPointVisibilityParams> frame_visibles;
  bool tracked = (TrackWithMotionModel(frame, frame_visibles) || TrackWithReferenceKeyFrame(frame));
//  bool tracked =TrackWithReferenceKeyFrame(frame);

  if (!tracked) {
    // TODO: go to relocalization
    velocity_is_valid_ = false;
    delete frame;
    return TrackingResult::TRACKING_FAILED;
  }
  ComputeVelocity(frame, last_frame_);


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
//  if(frame->GetMapPointCount() < 30)
//    return TrackingResult::TRACKING_FAILED;
  // =======================debug ======================
  cv::Mat image = cv::imread(frame->GetFilename());
  auto fr = dynamic_cast<frame::monocular::MonocularFrame *> (frame);

  if (mode & 1) {
    for (auto vmp: frame_visibles) {
      TPoint2D pt;
      fr->GetCamera()->ProjectAndDistort(fr->GetPosition().Transform(vmp.map_point->GetPosition()), pt);
      cv::circle(image, cv::Point2f(pt.x(), pt.y()), 3, cv::Scalar(0, 255, 255));
    }
    for (auto vmp: visible_map_points) {
      TPoint2D pt;
      fr->GetCamera()->ProjectAndDistort(fr->GetPosition().Transform(vmp.map_point->GetPosition()), pt);
      cv::circle(image, cv::Point2f(pt.x(), pt.y()), 3, cv::Scalar(0, 255, 255));
    }
  }

  if (mode & 2) {
    for (auto mp: fr->GetMapPoints()) {
      TPoint2D pt;
      fr->GetCamera()->ProjectAndDistort(fr->GetPosition().Transform(mp.second->GetPosition()), pt);
      cv::circle(image, cv::Point2f(pt.x(), pt.y()), 5, cv::Scalar(0, 255, 0));
    }
  }

  if (mode & 4) {
    for (auto mp: fr->GetBadMapPoints()) {
      TPoint2D pt;
      fr->GetCamera()->ProjectAndDistort(fr->GetPosition().Transform(mp.second->GetPosition()), pt);
      cv::circle(image, cv::Point2f(pt.x(), pt.y()), 2, cv::Scalar(0, 0, 255));
    }
  }

  char key = ']';
  if (mode & 8) {
    key = debug::DrawCommonMapPoints(fr->GetFilename(),
                                     last_frame_->GetFilename(),
                                     dynamic_cast<frame::monocular::MonocularFrame *>(fr),
                                     dynamic_cast<frame::monocular::MonocularFrame *>(last_frame_));
  }

  std::cout << "Frame Position after SLMP " << std::endl;
  frame->GetPosition().print();
  //std::cout << "Frame Position after SLMP " << frame->GetPosition() << std::endl;
  logging::RetrieveLogger()->debug("SLMP Local mp count {}", local_map_points_except_current.size());
  cv::imshow("After SLMP", image);

  key = cv::waitKey(time_to_wait);
  switch (key) {
    case 'c':time_to_wait = (int)!time_to_wait;
      break;
    case 'm':mode ^= 1;
      break;
    case 'v':mode ^= 2;
      break;
    case 'b':mode ^= 8;
      break;

  }

  std::ofstream ostream("mps/" + std::to_string(frame->Id()) + ".txt");
  for (auto mp: atlas_->GetCurrentMap()->GetAllMapPoints()) {
    ostream << mp->GetPosition().x() << "," << mp->GetPosition().y() << "," << mp->GetPosition().z() << std::endl;
  }

  //=====================================================

  frame::Frame::MapPointSet map_points;
  frame->ListMapPoints(map_points);
  for (auto map_point: map_points)
    map_point->IncreaseFound();

  //TODO: Add keyframe if necessary
  if (NeedNewKeyFrame(frame)) {
    auto keyframe = frame->CreateKeyFrame();
    last_key_frame_ = keyframe;
    atlas_->GetCurrentMap()->AddKeyFrame(keyframe);
    this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Update, .frame = keyframe});

    // TODO: remove the following line
  }
  (dynamic_cast<LocalMapper *>(*(observers_.begin())))->RunIteration();
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
  bool local_mapper_accepts_key_frame = true;
  float th_ref_ratio = 0.9f; // for monocular only
  bool more_than_min_frames_passed = frame->Id() >= last_key_frame_->Id() + MIN_FRAMES;
  std::unordered_set<map::MapPoint *> map_points;
  frame->ListMapPoints(map_points);
  size_t tracked_points_count = map_points.size();
  size_t min_observation_count = number_of_key_frames < 3 ? 2 : 3;

  std::unordered_set<map::MapPoint *> ref_kf_all_tracked_map_points;
  reference_keyframe_->ListMapPoints(ref_kf_all_tracked_map_points);
  std::unordered_set<map::MapPoint *> filtered_tracked_map_points;
  for(auto tracked_map_point : ref_kf_all_tracked_map_points) {
    if (tracked_map_point->GetObservationCount() >= min_observation_count) {
      filtered_tracked_map_points.insert(tracked_map_point);
    }
  }
  bool few_tracked_points =
      tracked_points_count < filtered_tracked_map_points.size() * th_ref_ratio && tracked_points_count > MIN_ACCEPTABLE_TRACKED_MAP_POINTS_COUNT;
  if (!few_tracked_points || (!more_than_max_frames_passed && !(more_than_min_frames_passed && local_mapper_accepts_key_frame))) {
    std::cout << "NeedNewKeyFrame = false1" << std::endl;
    return false;
  }
  std::cout << "NeedNewKeyFrame = " << local_mapper_accepts_key_frame << std::endl;
   return local_mapper_accepts_key_frame;
  /*std::unordered_set<map::MapPoint *> m;
  frame->ListMapPoints(m);
  bool need = kf_counter >= 4;// && m.size() > 40;
  if (need)
    kf_counter = 0;

  return need;*/
}

TrackingResult Tracker::TrackInFirstImageState(frame::Frame * frame) {

  assert(FIRST_IMAGE == state_);
  assert(frame->IsValid());

  if (frame->Link(last_frame_)) {
    map::Map * current_map = atlas_->GetCurrentMap();
    std::cout << "Position after linking " << std::endl;
    frame->GetPosition().print();

    frame::KeyFrame * initial_key_frame = last_frame_->CreateKeyFrame();
    initial_key_frame->SetInitial(true);
    frame::KeyFrame * current_key_frame = frame->CreateKeyFrame();
    last_key_frame_ = current_key_frame;

    current_map->SetInitialKeyFrame(initial_key_frame);
    current_map->AddKeyFrame(current_key_frame);

    std::unordered_set<frame::KeyFrame *> key_frames{initial_key_frame, current_key_frame};
    std::unordered_set<map::MapPoint *> map_points;
    current_key_frame->ListMapPoints(map_points);

    optimization::BundleAdjustment(key_frames, map_points, 30);
    std::cout << "Position after linking BA " << std::endl;
    current_key_frame->GetPosition().print();

    std::vector<precision_t> depths;
    for (auto mp: map_points) {
      current_map->AddMapPoint(mp);
      depths.push_back(mp->GetPosition().z());
    }

    auto pose = current_key_frame->GetPosition();
    pose.T /= depths[depths.size() / 2];
    current_key_frame->SetPosition(pose);
    frame->SetPosition(pose);

    std::sort(depths.begin(), depths.end());
    for (auto mp: map_points) {
      TPoint3D pose = mp->GetPosition();
      pose /= depths[depths.size() / 2];
      mp->min_invariance_distance_ /= depths[depths.size() / 2];
      mp->max_invariance_distance_ /= depths[depths.size() / 2];
      mp->SetPosition(pose);
      mp->Refresh(frame->GetFeatureExtractor());
    }

    reference_keyframe_ = current_key_frame;
    state_ = OK;
    last_frame_ = frame;

    this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Initial, .frame=initial_key_frame});
    this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Update, .frame=current_key_frame});


    // TODO: remove the following line in multithreading
    (dynamic_cast<LocalMapper *>(*(observers_.begin())))->RunIteration();

  } else {
    delete frame;
    return TrackingResult::LINKING_FAILED;
  }

  return TrackingResult::OK;
}

TrackingResult Tracker::TrackInNotInitializedState(frame::Frame * frame) {

  assert(NOT_INITIALIZED == state_);
  assert(frame->IsValid());
  frame->SetIdentity();
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
    default: {
    }
  }
  return res;
}

void Tracker::ComputeVelocity(const geometry::RigidObject * object2, const geometry::RigidObject * object1) {
  velocity_ = object1->GetInversePosition().T - object2->GetInversePosition().T;
  angular_velocity_ = object1->GetPosition().R * object2->GetInversePosition().R;
  velocity_is_valid_ = true;
}

void Tracker::PredictAndSetNewFramePosition(frame::Frame * frame) const {
  geometry::Pose pose;
  pose.R = angular_velocity_ * last_frame_->GetPosition().R;
  pose.T = -pose.R * (last_frame_->GetInversePosition().T + velocity_);
  frame->SetPosition(pose);
}

}  // namespace orb_slam3
