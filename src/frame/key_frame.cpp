//
// Created by vahagn on 06/07/2021.
//

#include "key_frame.h"
#include <map/map.h>
#include <map/atlas.h>
#include <frame/database/ikey_frame_database.h>
#include <settings.h>
#include <messages/messages.h>

namespace orb_slam3 {
namespace frame {

KeyFrame::KeyFrame(TimePoint time_point,
                   const std::string & filename,
                   const SensorConstants * sensor_constants,
                   size_t id,
                   map::Atlas * atlas)
    : BaseFrame(time_point, filename, sensor_constants, id, atlas),
      is_initialized_(false),
      covisibility_graph_(this),
      is_initial_(false),
      bad_flag_(false),
      kf_gba_(nullptr) {
  if (Settings::Get().MessageRequested(messages::MessageType::KEYFRAME_CREATED))
    messages::MessageProcessor::Instance().Enqueue(new messages::KeyFrameCreated(this));
}

KeyFrame::KeyFrame(std::istream & stream, serialization::SerializationContext & context) : BaseFrame(stream, context),
                                                                                           covisibility_graph_(this) {
  READ_FROM_STREAM(is_initial_, stream);
  READ_FROM_STREAM(is_initialized_, stream);
  READ_FROM_STREAM(bad_flag_, stream);
  if (Settings::Get().MessageRequested(messages::MessageType::KEYFRAME_CREATED))
    messages::MessageProcessor::Instance().Enqueue(new messages::KeyFrameCreated(this));
}

void KeyFrame::SetBad() {
  bad_flag_ = true;
  assert(nullptr != map_);
  map_->EraseKeyFrame(this);
  if (Settings::Get().MessageRequested(messages::MessageType::KEYFRAME_DELETED))
    messages::MessageProcessor::Instance().Enqueue(new messages::KeyFrameDeleted(this));
  atlas_->GetKeyframeDatabase()->Erase(this);
}

void KeyFrame::SetPoseGBA(const TMatrix33 & R, const TVector3D & T) {
  pose_gba_.R = R;
  pose_gba_.T = T;
}

void KeyFrame::ApplyStaging() {
  RigidObject::ApplyStaging();
  if (Settings::Get().MessageRequested(messages::KEYFRAME_POSITION_UPDATED))
    messages::MessageProcessor::Instance().Enqueue(new messages::KeyFramePositionUpdated(this));
}

CovisibilityGraphNode & KeyFrame::GetCovisibilityGraph() {
  return covisibility_graph_;
}

const CovisibilityGraphNode & KeyFrame::GetCovisibilityGraph() const {
  return covisibility_graph_;
}

bool KeyFrame::IsInitial() const { return is_initial_; }

void KeyFrame::SetInitial(bool initial) { is_initial_ = initial; }

bool KeyFrame::IsBad() const { return bad_flag_; }

void KeyFrame::Initialize() {
  if (is_initialized_)
    return;
  is_initialized_ = true;
  InitializeImpl();
}

void KeyFrame::AddMapPoint(Observation & observation) {
  this->AddMapPointImpl(observation);
  observation.GetMapPoint()->AddObservation(observation);
  if (Settings::Get().MessageRequested(messages::OBSERVATION_ADDED))
    messages::MessageProcessor::Instance().Enqueue(new messages::ObservationAdded(observation));
}

void KeyFrame::EraseMapPoint(map::MapPoint * map_point) {
  Observation observation;
  if (map_point->GetObservation(this, observation)) {
    EraseMapPointImpl(observation);
    map_point->EraseObservation(this);
    if (Settings::Get().MessageRequested(messages::OBSERVATION_DELETED))
      messages::MessageProcessor::Instance().Enqueue(new messages::ObservationDeleted(observation));
  } else
    assert(false);
}

void KeyFrame::SerializeToStream(std::ostream & stream) const {
  WRITE_TO_STREAM(is_initial_, stream);
  WRITE_TO_STREAM(is_initialized_, stream);
  WRITE_TO_STREAM(bad_flag_, stream);
}

}
}