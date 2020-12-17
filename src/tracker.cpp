//
// Created by vahagn on 12/8/20.
//

#include "tracker.h"
#include <constants.h>

namespace nvision {

TrackingResult Tracker::Track(const std::shared_ptr<FrameBase> &frame) {
  Map *current_map = atlas_->GetCurrentMap();

  if (current_map->FrameCount() == 0) {
    if (frame->FeatureCount() > MINIMAL_FEATURE_COUNT_PER_FRAME)
      frame->InitializeIdentity();

  }
  return OK;
}

}