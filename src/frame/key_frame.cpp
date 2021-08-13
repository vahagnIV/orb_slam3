//
// Created by vahagn on 06/07/2021.
//

#include "key_frame.h"
#include <map/map.h>
#include <settings.h>
#include <messages/messages.h>

namespace orb_slam3 {
namespace frame {

KeyFrame::KeyFrame(TimePoint time_point,
                   const std::string & filename,
                   const SensorConstants * sensor_constants,
                   size_t id,
                   std::shared_ptr<const features::handlers::BaseFeatureHandler> feature_handler)
    : BaseFrame(time_point, filename, sensor_constants, id, feature_handler),
      is_initialized_(false),
      covisibility_graph_(this),
      is_initial_(false),
      bad_flag_(false),
      kf_gba_(nullptr) {
  if (Settings::Get().MessageRequested(messages::MessageType::KEYFRAME_CREATED))
    messages::MessageProcessor::Instance().Enqueue(new messages::KeyFrameCreated(this));
}

void KeyFrame::SetBad() {
  bad_flag_ = true;
  assert(nullptr != map_);
  map_->EraseKeyFrame(this);
  if (Settings::Get().MessageRequested(messages::MessageType::KEYFRAME_DELETED))
    messages::MessageProcessor::Instance().Enqueue(new messages::KeyFrameDeleted(this));
}

void KeyFrame::SetPoseGBA(const TMatrix33 & R, const TVector3D & T) {
  pose_gba_.R = R;
  pose_gba_.T = T;
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

}
}