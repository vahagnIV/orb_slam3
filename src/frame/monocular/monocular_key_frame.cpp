//
// Created by vahagn on 11/05/2021.
//

#include "monocular_key_frame.h"
#include "monocular_frame.h"
#include <map/map_point.h>

namespace orb_slam3 {
namespace frame {
namespace monocular {

MonocularKeyFrame::MonocularKeyFrame(MonocularFrame * frame) : KeyFrame(frame->GetTimeCreated(),
                                                                        frame->GetFilename(),
                                                                        frame->GetFeatureExtractor(),
                                                                        frame->GetVocabulary(),
                                                                        frame->Id()),
                                                               BaseMonocular(*frame) {
  SetPosition(frame->GetPosition());
  for(auto mp: map_points_){
    mp.second->AddObservation(Observation(mp.second, this, mp.first));
  }
}

FrameType MonocularKeyFrame::Type() const {
  return MONOCULAR;
}

void MonocularKeyFrame::ListMapPoints(BaseFrame::MapPointSet & out_map_points) const {
  BaseMonocular::ListMapPoints(out_map_points);
}

TVector3D MonocularKeyFrame::GetNormal(const TPoint3D & point) const {
  TPoint3D normal = GetInversePosition().T - point;
  normal.normalize();
  return normal;
}

}
}
}