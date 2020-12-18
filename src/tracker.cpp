//
// Created by vahagn on 12/8/20.
//

#include "tracker.h"
#include <constants.h>

namespace nvision {

Tracker::Tracker() : atlas_(new Atlas) {

}

Tracker::~Tracker() {
  delete atlas_;
}

TrackingResult Tracker::Track(FrameBase *frame) {
  Map *current_map = atlas_->GetCurrentMap();

  if (current_map->GetLastFrame() == nullptr) {

    if (frame->FeatureCount() < MINIMAL_FEATURE_COUNT_PER_FRAME)
      return TrackingResult::Ignore;
    frame->InitializeIdentity();
    current_map->SetLastFrame(frame);
    current_map->AcceptLastFrame();

  } else {
    frame->SetReferenceFrame(current_map->GetLastFrame());
  }
  return OK;
}

}